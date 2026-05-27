/***** (C) Copyright, Sealien Robotics(Guangzhou) Co.,Ltd. ******source file****/
/*
 * adc_to_joy_mapper 实现.
 * 事件驱动: 由 /mixed_io/raw 订阅触发, 频率跟随 receiver (即 MCU 上报频率).
 * 单线程模型, 无锁; LPF 状态仅在 raw_callback 中修改.
 */

#include "sealien_mavlink_joy_bridge/adc_to_joy_mapper.hpp"

#include <algorithm>
#include <cmath>

/* 构造: 声明参数 -> 初始化 LPF -> 建发布器与订阅器 */
AdcToJoyMapper::AdcToJoyMapper(const std::string & node_name)
: Node(node_name),
  raw_topic_("/mixed_io/raw"),
  joy_topic_("/joy"),
  axes_count_(24U),
  buttons_count_(24U),
  adc_valid_min_v_(-0.5),
  adc_valid_max_v_(5.5)
{
    param_init();
    filtered_axes_.assign(axes_count_, 0.0f);

    joy_publisher_ = this->create_publisher<sensor_msgs::msg::Joy>(joy_topic_, 10);
    raw_subscriber_ = this->create_subscription<sealien_mavlink_joy_bridge::msg::MixedIoRaw>(
        raw_topic_, 10,
        std::bind(&AdcToJoyMapper::raw_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
        this->get_logger(),
        "adc_to_joy_mapper: %s -> %s (%zu axes, %zu buttons, valid V [%.2f, %.2f])",
        raw_topic_.c_str(),
        joy_topic_.c_str(),
        axes_count_,
        buttons_count_,
        adc_valid_min_v_,
        adc_valid_max_v_);
}

/* 声明并读取所有 ROS 参数, 默认值与 YAML 保持一致.
 * 数组长度不足 24 时按 def_val 补足; 多于 24 时截断. */
void AdcToJoyMapper::param_init()
{
    raw_topic_ = this->declare_parameter<std::string>("raw_topic", "/mixed_io/raw");
    joy_topic_ = this->declare_parameter<std::string>("joy_topic", "/joy");
    axes_count_ = static_cast<size_t>(this->declare_parameter<int>("axes_count", 24));
    buttons_count_ = static_cast<size_t>(this->declare_parameter<int>("buttons_count", 24));

    const size_t adc_n = 24U;
    // 通用 lambda: ROS 参数仅原生支持 double 数组, 这里转成 float 以减少滤波运算开销
    auto declare_v = [this, adc_n](const std::string & name, double def_val) {
        auto raw = this->declare_parameter<std::vector<double>>(
            name, std::vector<double>(adc_n, def_val));
        std::vector<float> out(adc_n, static_cast<float>(def_val));
        for (size_t i = 0U; i < adc_n && i < raw.size(); i++)
        {
            out[i] = static_cast<float>(raw[i]);
        }
        return out;
    };

    adc_min_v_ = declare_v("adc_min_v", 0.0);
    adc_center_v_ = declare_v("adc_center_v", 2.5);
    adc_max_v_ = declare_v("adc_max_v", 5.0);

    deadzone_low_ = this->declare_parameter<std::vector<double>>(
        "deadzone_low", std::vector<double>(adc_n, 0.05));
    deadzone_high_ = this->declare_parameter<std::vector<double>>(
        "deadzone_high", std::vector<double>(adc_n, 0.05));
    expo_ = this->declare_parameter<std::vector<double>>(
        "expo", std::vector<double>(adc_n, 0.0));
    invert_ = this->declare_parameter<std::vector<bool>>(
        "invert", std::vector<bool>(adc_n, false));
    lpf_alpha_ = this->declare_parameter<std::vector<double>>(
        "lpf_alpha", std::vector<double>(adc_n, 0.35));

    adc_valid_min_v_ = this->declare_parameter<double>("adc_valid_min_v", -0.5);
    adc_valid_max_v_ = this->declare_parameter<double>("adc_valid_max_v", 5.5);

    // 默认通道映射: 0..23 一一对应
    std::vector<int64_t> def_chans(adc_n);
    for (size_t i = 0U; i < adc_n; i++)
    {
        def_chans[i] = static_cast<int64_t>(i);
    }

    const auto axes_adc = this->declare_parameter<std::vector<int64_t>>(
        "axes_map_adc_channels", def_chans);
    const auto axes_joy = this->declare_parameter<std::vector<int64_t>>(
        "axes_map_joy_axes", def_chans);

    axes_map_.clear();
    const size_t axis_map_len = std::min(axes_adc.size(), axes_joy.size());
    for (size_t i = 0U; i < axis_map_len; i++)
    {
        adcjoy_axis_map_t entry {};
        entry.adc_channel = static_cast<uint8_t>(axes_adc[i]);
        entry.joy_axis = static_cast<uint8_t>(axes_joy[i]);
        axes_map_.push_back(entry);
    }

    const auto btn_gpio = this->declare_parameter<std::vector<int64_t>>(
        "buttons_gpio_indices", std::vector<int64_t>{});
    const auto btn_joy = this->declare_parameter<std::vector<int64_t>>(
        "buttons_joy_indices", std::vector<int64_t>{});
    const auto btn_active = this->declare_parameter<std::vector<bool>>(
        "buttons_active_high", std::vector<bool>{});

    button_map_.clear();
    const size_t btn_len = std::min(btn_gpio.size(), btn_joy.size());
    for (size_t i = 0U; i < btn_len; i++)
    {
        adcjoy_button_map_t entry {};
        entry.gpio_index = static_cast<uint8_t>(btn_gpio[i]);
        entry.joy_button = static_cast<uint8_t>(btn_joy[i]);
        // active_high 数组短于 gpio/joy 时, 缺省按高有效
        entry.active_high = (i < btn_active.size()) ? btn_active[i] : true;
        button_map_.push_back(entry);
    }
}

/* 断线/越界检测; 单独成函数, 便于复用与单测 */
bool AdcToJoyMapper::is_disconnected(float voltage_v) const
{
    if (std::isnan(voltage_v) || std::isinf(voltage_v))
    {
        return true;
    }
    return (voltage_v < static_cast<float>(adc_valid_min_v_)) ||
           (voltage_v > static_cast<float>(adc_valid_max_v_));
}

/* 单轴流水线: 电压 -> [-1,1]
 * 1) 三段线性标定: [vmin, vcenter] -> [-1, 0], [vcenter, vmax] -> [0, +1]
 * 2) 双边死区: 死区内输出 0, 死区外线性拉伸回端点
 * 3) Expo 曲线: out = sign * (expo*|x|^3 + (1-expo)*|x|)
 * 4) 通道反向
 * 注意: 不在此处做 LPF, 滤波由 apply_filter 独立完成.
 */
float AdcToJoyMapper::map_axis(uint8_t ch_index, float voltage_v, float * out_raw_centered) const
{
    if (out_raw_centered != nullptr)
    {
        *out_raw_centered = 0.0f;
    }

    if (ch_index >= adc_min_v_.size() || ch_index >= adc_center_v_.size() ||
        ch_index >= adc_max_v_.size())
    {
        return 0.0f;
    }

    const float vmin = adc_min_v_[ch_index];
    const float vmax = adc_max_v_[ch_index];
    const float vcenter = adc_center_v_[ch_index];

    // step 1: 三段线性标定
    float centered = 0.0f;
    if (voltage_v >= vcenter)
    {
        const float denom = vmax - vcenter;
        if (denom > 0.001f)
        {
            centered = (voltage_v - vcenter) / denom;
        }
    }
    else
    {
        const float denom = vcenter - vmin;
        if (denom > 0.001f)
        {
            centered = (voltage_v - vcenter) / denom;
        }
    }
    centered = ADCJOY_LIMIT(centered, -1.0f, 1.0f);

    if (out_raw_centered != nullptr)
    {
        *out_raw_centered = centered;
    }

    // step 2: 双边死区 (clamp 上界避免除零)
    float dz_low = 0.05f;
    float dz_high = 0.05f;
    if (ch_index < deadzone_low_.size())
    {
        dz_low = static_cast<float>(deadzone_low_[ch_index]);
    }
    if (ch_index < deadzone_high_.size())
    {
        dz_high = static_cast<float>(deadzone_high_[ch_index]);
    }
    dz_low = ADCJOY_LIMIT(dz_low, 0.0f, 0.999f);
    dz_high = ADCJOY_LIMIT(dz_high, 0.0f, 0.999f);

    float after_dz = centered;
    if (centered > 0.0f)
    {
        if (centered <= dz_high)
        {
            after_dz = 0.0f;
        }
        else
        {
            after_dz = (centered - dz_high) / (1.0f - dz_high);
        }
    }
    else if (centered < 0.0f)
    {
        if (-centered <= dz_low)
        {
            after_dz = 0.0f;
        }
        else
        {
            after_dz = (centered + dz_low) / (1.0f - dz_low);
        }
    }
    after_dz = ADCJOY_LIMIT(after_dz, -1.0f, 1.0f);

    // step 3: Expo 曲线 (保持符号)
    float expo = 0.0f;
    if (ch_index < expo_.size())
    {
        expo = static_cast<float>(expo_[ch_index]);
    }
    expo = ADCJOY_LIMIT(expo, 0.0f, 1.0f);

    const float abs_v = std::fabs(after_dz);
    const float sign_v = (after_dz >= 0.0f) ? 1.0f : -1.0f;
    const float curved = sign_v * (expo * abs_v * abs_v * abs_v + (1.0f - expo) * abs_v);

    // step 4: 通道反向
    float result = curved;
    if (ch_index < invert_.size() && invert_[ch_index])
    {
        result = -result;
    }

    return result;
}

/* 一阶低通: filtered = alpha*new + (1-alpha)*prev.
 * alpha=1 时无滤波, alpha=0 时锁死在 prev. */
float AdcToJoyMapper::apply_filter(uint8_t joy_axis, uint8_t ch_index, float new_value)
{
    if (joy_axis >= filtered_axes_.size())
    {
        return new_value;
    }
    float alpha = 0.35f;
    if (ch_index < lpf_alpha_.size())
    {
        alpha = static_cast<float>(lpf_alpha_[ch_index]);
    }
    alpha = ADCJOY_LIMIT(alpha, 0.0f, 1.0f);
    const float prev = filtered_axes_[joy_axis];
    const float filtered = alpha * new_value + (1.0f - alpha) * prev;
    filtered_axes_[joy_axis] = filtered;
    return filtered;
}

/* 发布全 0 Joy 并清 LPF; 避免链路恢复后残留旧滤波值 */
void AdcToJoyMapper::publish_failsafe()
{
    sensor_msgs::msg::Joy joy_msg;
    joy_msg.header.stamp = this->now();
    joy_msg.header.frame_id = "mixed_io_failsafe";
    joy_msg.axes.assign(axes_count_, 0.0);
    joy_msg.buttons.assign(buttons_count_, 0);
    std::fill(filtered_axes_.begin(), filtered_axes_.end(), 0.0f);
    joy_publisher_->publish(joy_msg);
}

/* /mixed_io/raw 回调: link_ok=false 直接 failsafe; 否则按 axes_map_ / button_map_ 生成 Joy */
void AdcToJoyMapper::raw_callback(const sealien_mavlink_joy_bridge::msg::MixedIoRaw & msg)
{
    // 运行期 axes_count_ 不会变, 但参数更新场景下保护一下
    if (filtered_axes_.size() != axes_count_)
    {
        filtered_axes_.assign(axes_count_, 0.0f);
    }

    if (!msg.link_ok)
    {
        publish_failsafe();
        return;
    }

    sensor_msgs::msg::Joy joy_msg;
    joy_msg.header.stamp = this->now();
    joy_msg.header.frame_id = "mixed_io";
    joy_msg.axes.assign(axes_count_, 0.0);
    joy_msg.buttons.assign(buttons_count_, 0);

    for (const auto & entry : axes_map_)
    {
        if (entry.adc_channel >= msg.adc_v.size() || entry.joy_axis >= joy_msg.axes.size())
        {
            continue;
        }

        const float v = msg.adc_v[entry.adc_channel];
        float axis_value = 0.0f;
        if (is_disconnected(v))
        {
            // 断线: 强制 0 + 清 LPF, 避免恢复瞬间从旧值缓慢爬出
            axis_value = 0.0f;
            filtered_axes_[entry.joy_axis] = 0.0f;
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "ADC ch%u disconnected: V=%.3f out of [%.2f, %.2f]",
                entry.adc_channel,
                v,
                adc_valid_min_v_,
                adc_valid_max_v_);
        }
        else
        {
            float raw_centered = 0.0f;
            const float mapped = map_axis(entry.adc_channel, v, &raw_centered);
            axis_value = apply_filter(entry.joy_axis, entry.adc_channel, mapped);
        }
        joy_msg.axes[entry.joy_axis] = static_cast<double>(axis_value);
    }

    for (const auto & entry : button_map_)
    {
        // gpio_index 上限固定 64 (msg.gpio_input_mask 是 uint64)
        if (entry.gpio_index >= 64U || entry.joy_button >= joy_msg.buttons.size())
        {
            continue;
        }
        const bool raw = ((msg.gpio_input_mask >> entry.gpio_index) & 1U) != 0U;
        const bool pressed = entry.active_high ? raw : !raw;
        joy_msg.buttons[entry.joy_button] = pressed ? 1 : 0;
    }

    joy_publisher_->publish(joy_msg);
}
