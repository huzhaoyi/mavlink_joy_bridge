/***** (C) Copyright, Sealien Robotics(Guangzhou) Co.,Ltd. ******source file****/

#include "sealien_mavlink_joy_bridge/mixed_io_joy_bridge_core.hpp"

#include <arpa/inet.h>
#include <cmath>
#include <cerrno>
#include <cstring>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "sealien_mavlink/mavlink.h"

#define MIXED_IO_MAVLINK_COMM (0U)
#define MIXED_IO_UDP_BUF_SIZE (2048)

static mavlink_status_t g_mavlink_status;
static mavlink_message_t g_mavlink_message;

static uint64_t pack_gpio_input_mask(uint32_t gpio_lo, uint32_t gpio_hi)
{
    return (static_cast<uint64_t>(gpio_hi) << 32) | static_cast<uint64_t>(gpio_lo);
}

MixedIoJoyBridge::MixedIoJoyBridge(const std::string & node_name)
: Node(node_name),
  listen_port_(14550),
  joy_topic_("/joy"),
  publish_rate_hz_(50.0),
  link_timeout_ms_(500),
  axes_count_(24U),
  buttons_count_(24U),
  udp_socket_fd_(-1),
  thread_running_(false),
  link_was_ok_(false),
  has_latest_(false),
  gpio_input_mask_(0U)
{
    for (uint8_t i = 0U; i < MIXED_IO_ADC_COUNT; i++)
    {
        adc_v_[i] = 0.0f;
    }
    last_rx_time_ = std::chrono::steady_clock::now();
    param_init();

    filtered_axes_.assign(axes_count_, 0.0f);

    joy_publisher_ = this->create_publisher<sensor_msgs::msg::Joy>(joy_topic_, 10);

    thread_running_.store(true);
    udp_thread_ = std::thread(&MixedIoJoyBridge::udp_thread_func, this);

    const auto period_ms = static_cast<int64_t>(1000.0 / publish_rate_hz_);
    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_ms),
        std::bind(&MixedIoJoyBridge::timer_callback, this));

    RCLCPP_INFO(
        this->get_logger(),
        "mixed_io_joy_bridge: listen %s:%d, publish %s (%zu axes, %zu buttons, %u ADC, %u GPIO)",
        listen_address_.c_str(),
        listen_port_,
        joy_topic_.c_str(),
        axes_count_,
        buttons_count_,
        MIXED_IO_ADC_COUNT,
        MIXED_IO_GPIO_COUNT);
}

MixedIoJoyBridge::~MixedIoJoyBridge()
{
    thread_running_.store(false);
    close_udp_socket();
    if (udp_thread_.joinable())
    {
        udp_thread_.join();
    }
}

void MixedIoJoyBridge::param_init()
{
    listen_address_ = this->declare_parameter<std::string>("listen_address", "0.0.0.0");
    listen_port_ = this->declare_parameter<int>("listen_port", 14550);
    remote_ip_filter_ = this->declare_parameter<std::string>("remote_ip_filter", "");
    joy_topic_ = this->declare_parameter<std::string>("joy_topic", "/joy");
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 50.0);
    link_timeout_ms_ = this->declare_parameter<int>("link_timeout_ms", 500);
    axes_count_ = static_cast<size_t>(this->declare_parameter<int>("axes_count", 24));
    buttons_count_ = static_cast<size_t>(this->declare_parameter<int>("buttons_count", 24));

    const auto adc_min_in = this->declare_parameter<std::vector<double>>(
        "adc_min_v", std::vector<double>(MIXED_IO_ADC_COUNT, 0.0));
    const auto adc_max_in = this->declare_parameter<std::vector<double>>(
        "adc_max_v", std::vector<double>(MIXED_IO_ADC_COUNT, 5.0));

    adc_min_v_.assign(MIXED_IO_ADC_COUNT, 0.0f);
    adc_max_v_.assign(MIXED_IO_ADC_COUNT, 5.0f);
    for (size_t i = 0U; i < MIXED_IO_ADC_COUNT && i < adc_min_in.size(); i++)
    {
        adc_min_v_[i] = static_cast<float>(adc_min_in[i]);
    }
    for (size_t i = 0U; i < MIXED_IO_ADC_COUNT && i < adc_max_in.size(); i++)
    {
        adc_max_v_[i] = static_cast<float>(adc_max_in[i]);
    }

    deadzone_ = this->declare_parameter<std::vector<double>>(
        "deadzone", std::vector<double>(MIXED_IO_ADC_COUNT, 0.05));
    invert_ = this->declare_parameter<std::vector<bool>>(
        "invert", std::vector<bool>(MIXED_IO_ADC_COUNT, false));
    lpf_alpha_ = this->declare_parameter<std::vector<double>>(
        "lpf_alpha", std::vector<double>(MIXED_IO_ADC_COUNT, 0.35));

    const auto axes_adc = this->declare_parameter<std::vector<int64_t>>(
        "axes_map_adc_channels", {0, 1, 2, 3});
    const auto axes_joy = this->declare_parameter<std::vector<int64_t>>(
        "axes_map_joy_axes", {0, 1, 2, 3});

    axes_map_.clear();
    const size_t axis_map_len = std::min(axes_adc.size(), axes_joy.size());
    for (size_t i = 0U; i < axis_map_len; i++)
    {
        mixed_io_axis_map_t entry {};
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
        mixed_io_button_map_t entry {};
        entry.gpio_index = static_cast<uint8_t>(btn_gpio[i]);
        entry.joy_button = static_cast<uint8_t>(btn_joy[i]);
        entry.active_high = true;
        if (i < btn_active.size())
        {
            entry.active_high = btn_active[i];
        }
        button_map_.push_back(entry);
    }
}

void MixedIoJoyBridge::close_udp_socket()
{
    if (udp_socket_fd_ >= 0)
    {
        close(udp_socket_fd_);
        udp_socket_fd_ = -1;
    }
}

float MixedIoJoyBridge::normalize_voltage(uint8_t ch_index, float voltage_v) const
{
    if (ch_index >= adc_min_v_.size() || ch_index >= adc_max_v_.size())
    {
        return 0.0f;
    }

    const float vmin = adc_min_v_[ch_index];
    const float vmax = adc_max_v_[ch_index];
    const float span = vmax - vmin;

    float norm = 0.0f;
    if (span > 0.001f)
    {
        norm = (voltage_v - vmin) / span;
    }

    norm = MIXED_IO_LIMIT(norm, 0.0f, 1.0f);

    float dz = 0.05f;
    if (ch_index < deadzone_.size())
    {
        dz = static_cast<float>(deadzone_[ch_index]);
    }
    if (norm < dz)
    {
        norm = 0.0f;
    }

    if (ch_index < invert_.size() && invert_[ch_index])
    {
        norm = 1.0f - norm;
    }

    return norm;
}

void MixedIoJoyBridge::udp_thread_func()
{
    udp_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_socket_fd_ < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "socket() failed: %s", strerror(errno));
        return;
    }

    int reuse = 1;
    setsockopt(udp_socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(listen_port_));
    if (inet_pton(AF_INET, listen_address_.c_str(), &addr.sin_addr) <= 0)
    {
        RCLCPP_ERROR(this->get_logger(), "invalid listen_address");
        close_udp_socket();
        return;
    }

    if (bind(udp_socket_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "bind() failed: %s", strerror(errno));
        close_udp_socket();
        return;
    }

    uint8_t buf[MIXED_IO_UDP_BUF_SIZE];
    while (thread_running_.load())
    {
        struct sockaddr_in from_addr {};
        socklen_t from_len = sizeof(from_addr);
        const ssize_t n = recvfrom(
            udp_socket_fd_,
            buf,
            sizeof(buf),
            0,
            reinterpret_cast<struct sockaddr *>(&from_addr),
            &from_len);

        if (n < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
            {
                continue;
            }
            if (!thread_running_.load())
            {
                break;
            }
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "recvfrom: %s",
                strerror(errno));
            continue;
        }

        if (!remote_ip_filter_.empty())
        {
            char ip_str[INET_ADDRSTRLEN] = {0};
            inet_ntop(AF_INET, &from_addr.sin_addr, ip_str, sizeof(ip_str));
            if (remote_ip_filter_ != ip_str)
            {
                continue;
            }
        }

        for (ssize_t i = 0; i < n; i++)
        {
            if (mavlink_parse_char(
                    MIXED_IO_MAVLINK_COMM,
                    buf[i],
                    &g_mavlink_message,
                    &g_mavlink_status) != MAVLINK_FRAMING_OK)
            {
                continue;
            }

            if (g_mavlink_message.msgid != MAVLINK_MSG_ID_MIXED_IO_DATA)
            {
                continue;
            }

            mavlink_mixed_io_data_t io_data {};
            mavlink_msg_mixed_io_data_decode(&g_mavlink_message, &io_data);

            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                for (uint8_t k = 0U; k < 8U; k++)
                {
                    adc_v_[k] = io_data.adc_dev1_Vin[k];
                    adc_v_[8U + k] = io_data.adc_dev2_Vin[k];
                    adc_v_[16U + k] = io_data.adc_dev3_Vin[k];
                }
                gpio_input_mask_ = pack_gpio_input_mask(
                    io_data.gpio_1_32_input,
                    io_data.gpio_33_64_input);
                has_latest_ = true;
                last_rx_time_ = std::chrono::steady_clock::now();
            }

            if (!link_was_ok_)
            {
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    5000,
                    "MIXED_IO_DATA rx (msgid=20)");
            }
        }
    }
}

void MixedIoJoyBridge::publish_joy()
{
    sensor_msgs::msg::Joy joy_msg;
    joy_msg.header.stamp = this->now();
    joy_msg.header.frame_id = "mixed_io";
    joy_msg.axes.assign(axes_count_, 0.0);
    joy_msg.buttons.assign(buttons_count_, 0);

    float adc_copy[MIXED_IO_ADC_COUNT] = {0.0f};
    uint64_t gpio_copy = 0U;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        for (uint8_t i = 0U; i < MIXED_IO_ADC_COUNT; i++)
        {
            adc_copy[i] = adc_v_[i];
        }
        gpio_copy = gpio_input_mask_;
    }

    if (filtered_axes_.size() < axes_count_)
    {
        filtered_axes_.assign(axes_count_, 0.0f);
    }

    for (const auto & entry : axes_map_)
    {
        if (entry.adc_channel >= MIXED_IO_ADC_COUNT)
        {
            continue;
        }
        if (entry.joy_axis >= joy_msg.axes.size())
        {
            continue;
        }

        const float norm = normalize_voltage(entry.adc_channel, adc_copy[entry.adc_channel]);

        float alpha = 0.35f;
        if (entry.adc_channel < lpf_alpha_.size())
        {
            alpha = static_cast<float>(lpf_alpha_[entry.adc_channel]);
        }
        alpha = MIXED_IO_LIMIT(alpha, 0.0f, 1.0f);

        const float prev = filtered_axes_[entry.joy_axis];
        const float filtered = alpha * norm + (1.0f - alpha) * prev;
        filtered_axes_[entry.joy_axis] = filtered;
        joy_msg.axes[entry.joy_axis] = static_cast<double>(filtered);
    }

    for (const auto & entry : button_map_)
    {
        if (entry.gpio_index >= MIXED_IO_GPIO_COUNT)
        {
            continue;
        }
        if (entry.joy_button >= joy_msg.buttons.size())
        {
            continue;
        }

        const bool raw = (gpio_copy >> entry.gpio_index) & 1U;
        const bool pressed = entry.active_high ? raw : !raw;
        joy_msg.buttons[entry.joy_button] = pressed ? 1 : 0;
    }

    joy_publisher_->publish(joy_msg);
}

void MixedIoJoyBridge::timer_callback()
{
    if (!has_latest_)
    {
        return;
    }

    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - last_rx_time_).count();
    const bool link_ok = elapsed_ms <= static_cast<int64_t>(link_timeout_ms_);

    if (!link_ok)
    {
        if (link_was_ok_)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "MIXED_IO_DATA timeout (%d ms), stop publishing %s",
                link_timeout_ms_,
                joy_topic_.c_str());
            link_was_ok_ = false;
        }
        return;
    }

    if (!link_was_ok_)
    {
        link_was_ok_ = true;
        RCLCPP_INFO(this->get_logger(), "MIXED_IO_DATA -> Joy link active");
    }

    publish_joy();
}
