/***** (C) Copyright, Sealien Robotics(Guangzhou) Co.,Ltd. ******source file****/

#include "sealien_mavlink_joy_bridge/adc_joy_mapper_core.hpp"

#include <cmath>

#define AJOY_ZORRO_AXES (7U)
#define AJOY_ZORRO_BUTTONS (24U)
#define AJOY_MODE_AXIS (6U)
#define AJOY_PWM_CENTER (1500U)

AdcJoyMapper::AdcJoyMapper(const std::string & node_name)
: Node(node_name),
  publish_rate_hz_(50.0),
  link_timeout_ms_(500),
  mode_adc_channel_(-1),
  axes_count_(AJOY_ZORRO_AXES),
  buttons_count_(AJOY_ZORRO_BUTTONS),
  has_adc_(false),
  link_was_ok_(false)
{
    last_adc_time_ = std::chrono::steady_clock::now();
    param_init();

    adc_subscriber_ = this->create_subscription<sealien_mavlink_joy_bridge::msg::AdcRaw>(
        adc_topic_,
        10,
        std::bind(&AdcJoyMapper::adc_callback, this, std::placeholders::_1));

    joy_publisher_ = this->create_publisher<sensor_msgs::msg::Joy>(joy_topic_, 10);

    const auto period_ms = static_cast<int64_t>(1000.0 / publish_rate_hz_);
    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_ms),
        std::bind(&AdcJoyMapper::timer_callback, this));

    filtered_axes_.assign(axes_count_, 0.0f);

    RCLCPP_INFO(
        this->get_logger(),
        "adc_joy_mapper: %s -> %s (%zu axes, %zu buttons)",
        adc_topic_.c_str(),
        joy_topic_.c_str(),
        axes_count_,
        buttons_count_);
}

void AdcJoyMapper::param_init()
{
    adc_topic_ = this->declare_parameter<std::string>("adc_topic", "adc/raw");
    joy_topic_ = this->declare_parameter<std::string>("joy_topic", "/joy");
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 50.0);
    link_timeout_ms_ = this->declare_parameter<int>("link_timeout_ms", 500);
    mode_adc_channel_ = this->declare_parameter<int>("mode_adc_channel", -1);

    const auto adc_min_in = this->declare_parameter<std::vector<int64_t>>(
        "adc_min", {1000, 1000, 1000, 1000});
    const auto adc_max_in = this->declare_parameter<std::vector<int64_t>>(
        "adc_max", {2000, 2000, 2000, 2000});
    const auto adc_center_in = this->declare_parameter<std::vector<int64_t>>(
        "adc_center", {1500, 1500, 1500, 1500});

    adc_min_.clear();
    adc_max_.clear();
    adc_center_.clear();
    for (size_t i = 0U; i < adc_min_in.size(); i++)
    {
        adc_min_.push_back(static_cast<uint16_t>(adc_min_in[i]));
    }
    for (size_t i = 0U; i < adc_max_in.size(); i++)
    {
        adc_max_.push_back(static_cast<uint16_t>(adc_max_in[i]));
    }
    for (size_t i = 0U; i < adc_center_in.size(); i++)
    {
        adc_center_.push_back(static_cast<uint16_t>(adc_center_in[i]));
    }
    deadzone_ = this->declare_parameter<std::vector<double>>("deadzone", {0.05, 0.05, 0.05, 0.05});
    invert_ = this->declare_parameter<std::vector<bool>>("invert", {false, false, false, false});
    lpf_alpha_ = this->declare_parameter<std::vector<double>>("lpf_alpha", {0.35, 0.35, 0.35, 0.35});
    mode_positions_ = this->declare_parameter<std::vector<double>>(
        "mode_positions", {-1.0, 0.0, 1.0});

    const auto axes_adc = this->declare_parameter<std::vector<int64_t>>(
        "axes_map_adc_channels", {0, 1, 2, 3});
    const auto axes_joy = this->declare_parameter<std::vector<int64_t>>(
        "axes_map_joy_axes", {0, 1, 2, 3});

    axes_map_.clear();
    const size_t map_len = std::min(axes_adc.size(), axes_joy.size());
    for (size_t i = 0U; i < map_len; i++)
    {
        ajoy_axis_map_t entry {};
        entry.adc_channel = static_cast<uint8_t>(axes_adc[i]);
        entry.joy_axis = static_cast<uint8_t>(axes_joy[i]);
        axes_map_.push_back(entry);
    }

    const auto btn_adc = this->declare_parameter<std::vector<int64_t>>(
        "buttons_adc_channels", std::vector<int64_t>{});
    const auto btn_joy = this->declare_parameter<std::vector<int64_t>>(
        "buttons_joy_indices", std::vector<int64_t>{});
    const auto btn_thr = this->declare_parameter<std::vector<int64_t>>(
        "buttons_thresholds", std::vector<int64_t>{});

    button_map_.clear();
    const size_t btn_len = std::min(btn_adc.size(), btn_joy.size());
    for (size_t i = 0U; i < btn_len; i++)
    {
        ajoy_button_map_t entry {};
        entry.adc_channel = static_cast<uint8_t>(btn_adc[i]);
        entry.joy_button = static_cast<uint8_t>(btn_joy[i]);
        entry.threshold = AJOY_PWM_CENTER;
        if (i < btn_thr.size())
        {
            entry.threshold = static_cast<uint16_t>(btn_thr[i]);
        }
        button_map_.push_back(entry);
    }
}

void AdcJoyMapper::adc_callback(const sealien_mavlink_joy_bridge::msg::AdcRaw & msg)
{
    latest_adc_ = msg;
    has_adc_ = true;
    last_adc_time_ = std::chrono::steady_clock::now();
}

float AdcJoyMapper::normalize_channel(uint8_t ch_index, uint16_t raw_us) const
{
    if (ch_index >= adc_min_.size() || ch_index >= adc_max_.size() || ch_index >= adc_center_.size())
    {
        return 0.0f;
    }

    const float raw = static_cast<float>(raw_us);
    const float vmin = static_cast<float>(adc_min_[ch_index]);
    const float vmax = static_cast<float>(adc_max_[ch_index]);
    const float vcenter = static_cast<float>(adc_center_[ch_index]);

    float norm = 0.0f;
    if (raw >= vcenter)
    {
        const float denom = vmax - vcenter;
        if (denom > 1.0f)
        {
            norm = (raw - vcenter) / denom;
        }
    }
    else
    {
        const float denom = vcenter - vmin;
        if (denom > 1.0f)
        {
            norm = (raw - vcenter) / denom;
        }
    }

    norm = AJOY_LIMIT(norm, -1.0f, 1.0f);

    float dz = 0.05f;
    if (ch_index < deadzone_.size())
    {
        dz = static_cast<float>(deadzone_[ch_index]);
    }
    if (std::fabs(norm) < dz)
    {
        norm = 0.0f;
    }

    if (ch_index < invert_.size() && invert_[ch_index])
    {
        norm = -norm;
    }

    return norm;
}

float AdcJoyMapper::apply_mode_from_adc(uint16_t raw_us) const
{
    if (mode_adc_channel_ < 0)
    {
        return 0.0f;
    }

    const float norm = normalize_channel(static_cast<uint8_t>(mode_adc_channel_), raw_us);
    if (mode_positions_.size() >= 3U)
    {
        const float low = static_cast<float>(mode_positions_[0]);
        const float mid = static_cast<float>(mode_positions_[1]);
        const float high = static_cast<float>(mode_positions_[2]);
        if (norm < -0.33f)
        {
            return low;
        }
        if (norm > 0.33f)
        {
            return high;
        }
        return mid;
    }
    return 0.0f;
}

void AdcJoyMapper::publish_joy()
{
    sensor_msgs::msg::Joy joy_msg;
    joy_msg.header.stamp = this->now();
    joy_msg.header.frame_id = "zorro";
    joy_msg.axes.assign(axes_count_, 0.0);
    joy_msg.buttons.assign(buttons_count_, 0);

    for (const auto & entry : axes_map_)
    {
        if (entry.adc_channel >= latest_adc_.channels.size())
        {
            continue;
        }
        if (entry.joy_axis >= joy_msg.axes.size())
        {
            continue;
        }

        const float norm = normalize_channel(
            entry.adc_channel,
            latest_adc_.channels[entry.adc_channel]);

        float alpha = 0.35f;
        if (entry.adc_channel < lpf_alpha_.size())
        {
            alpha = static_cast<float>(lpf_alpha_[entry.adc_channel]);
        }
        alpha = AJOY_LIMIT(alpha, 0.0f, 1.0f);

        const float prev = filtered_axes_[entry.joy_axis];
        const float filtered = alpha * norm + (1.0f - alpha) * prev;
        filtered_axes_[entry.joy_axis] = filtered;
        joy_msg.axes[entry.joy_axis] = static_cast<double>(filtered);
    }

    for (const auto & entry : button_map_)
    {
        if (entry.adc_channel >= latest_adc_.channels.size())
        {
            continue;
        }
        if (entry.joy_button >= joy_msg.buttons.size())
        {
            continue;
        }
        joy_msg.buttons[entry.joy_button] =
            (latest_adc_.channels[entry.adc_channel] > entry.threshold) ? 1 : 0;
    }

    if (mode_adc_channel_ >= 0 &&
        static_cast<size_t>(mode_adc_channel_) < latest_adc_.channels.size() &&
        AJOY_MODE_AXIS < joy_msg.axes.size())
    {
        joy_msg.axes[AJOY_MODE_AXIS] = static_cast<double>(
            apply_mode_from_adc(latest_adc_.channels[static_cast<size_t>(mode_adc_channel_)]));
    }
    else if (latest_adc_.mode != 127)
    {
        if (latest_adc_.mode <= -1)
        {
            joy_msg.axes[AJOY_MODE_AXIS] = -1.0;
        }
        else if (latest_adc_.mode >= 1)
        {
            joy_msg.axes[AJOY_MODE_AXIS] = 1.0;
        }
        else
        {
            joy_msg.axes[AJOY_MODE_AXIS] = 0.0;
        }
    }

    joy_publisher_->publish(joy_msg);
}

void AdcJoyMapper::timer_callback()
{
    if (!has_adc_)
    {
        return;
    }

    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - last_adc_time_).count();
    const bool link_ok = (latest_adc_.link_ok != 0U) &&
        (elapsed_ms <= static_cast<int64_t>(link_timeout_ms_));

    if (!link_ok)
    {
        if (link_was_ok_)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "ADC link timeout (%d ms), stop publishing %s",
                link_timeout_ms_,
                joy_topic_.c_str());
            link_was_ok_ = false;
        }
        return;
    }

    if (!link_was_ok_)
    {
        link_was_ok_ = true;
        RCLCPP_INFO(this->get_logger(), "ADC -> Joy link active");
    }

    publish_joy();
}
