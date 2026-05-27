/***** (C) Copyright, Sealien Robotics(Guangzhou) Co.,Ltd. ******source file****/

#include "sealien_mavlink_joy_bridge/io_joy_bridge_core.hpp"

#include <arpa/inet.h>
#include <cmath>
#include <cerrno>
#include <cstring>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "mavlink.h"

#define IOJOY_MAVLINK_COMM (0U)
#define IOJOY_UDP_BUF_SIZE (2048)

static mavlink_status_t g_mavlink_status;
static mavlink_message_t g_mavlink_message;

static float apply_deadzone(float v, float deadzone)
{
    if (std::fabs(v) < deadzone)
    {
        return 0.0f;
    }
    return v;
}

IoJoyBridge::IoJoyBridge(const std::string & node_name)
: Node(node_name),
  listen_port_(14550),
  joy_topic_("/joy"),
  publish_rate_hz_(50.0),
  link_timeout_ms_(500),
  deadzone_(0.05),
  udp_socket_fd_(-1),
  thread_running_(false),
  link_was_ok_(false),
  has_latest_(false),
  di_mask_(0U)
{
    for (uint8_t i = 0U; i < IO_JOY_AI_COUNT; i++)
    {
        ai_[i] = 0.0f;
    }
    last_rx_time_ = std::chrono::steady_clock::now();
    param_init();

    joy_publisher_ = this->create_publisher<sensor_msgs::msg::Joy>(joy_topic_, 10);

    thread_running_.store(true);
    udp_thread_ = std::thread(&IoJoyBridge::udp_thread_func, this);

    const auto period_ms = static_cast<int64_t>(1000.0 / publish_rate_hz_);
    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_ms),
        std::bind(&IoJoyBridge::timer_callback, this));

    RCLCPP_INFO(
        this->get_logger(),
        "io_joy_bridge: listen %s:%d, publish %s (%u AI -> axes, %u DI -> buttons)",
        listen_address_.c_str(),
        listen_port_,
        joy_topic_.c_str(),
        IO_JOY_AI_COUNT,
        IO_JOY_DI_COUNT);
}

IoJoyBridge::~IoJoyBridge()
{
    thread_running_.store(false);
    close_udp_socket();
    if (udp_thread_.joinable())
    {
        udp_thread_.join();
    }
}

void IoJoyBridge::param_init()
{
    listen_address_ = this->declare_parameter<std::string>("listen_address", "0.0.0.0");
    listen_port_ = this->declare_parameter<int>("listen_port", 14550);
    remote_ip_filter_ = this->declare_parameter<std::string>("remote_ip_filter", "");
    joy_topic_ = this->declare_parameter<std::string>("joy_topic", "/joy");
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 50.0);
    link_timeout_ms_ = this->declare_parameter<int>("link_timeout_ms", 500);
    deadzone_ = this->declare_parameter<double>("deadzone", 0.05);
}

void IoJoyBridge::close_udp_socket()
{
    if (udp_socket_fd_ >= 0)
    {
        close(udp_socket_fd_);
        udp_socket_fd_ = -1;
    }
}

void IoJoyBridge::udp_thread_func()
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

    uint8_t buf[IOJOY_UDP_BUF_SIZE];
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
                    IOJOY_MAVLINK_COMM,
                    buf[i],
                    &g_mavlink_message,
                    &g_mavlink_status) != MAVLINK_FRAMING_OK)
            {
                continue;
            }

            if (g_mavlink_message.msgid != MAVLINK_MSG_ID_SEALIEN_IO_INPUT)
            {
                continue;
            }

            mavlink_sealien_io_input_t io_in {};
            mavlink_msg_sealien_io_input_decode(&g_mavlink_message, &io_in);

            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                for (uint8_t k = 0U; k < IO_JOY_AI_COUNT; k++)
                {
                    float v = io_in.ai[k];
                    if (v < -1.0f)
                    {
                        v = -1.0f;
                    }
                    if (v > 1.0f)
                    {
                        v = 1.0f;
                    }
                    ai_[k] = v;
                }
                di_mask_ = io_in.di & 0x00FFFFFFU;
                has_latest_ = true;
                last_rx_time_ = std::chrono::steady_clock::now();
            }

            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "SEALIEN_IO_INPUT rx (msgid=203)");
        }
    }
}

void IoJoyBridge::publish_joy()
{
    sensor_msgs::msg::Joy joy_msg;
    joy_msg.header.stamp = this->now();
    joy_msg.header.frame_id = "sealien_io";
    joy_msg.axes.assign(IO_JOY_AI_COUNT, 0.0);
    joy_msg.buttons.assign(IO_JOY_DI_COUNT, 0);

    float ai_copy[IO_JOY_AI_COUNT] = {0.0f};
    uint32_t di_copy = 0U;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        for (uint8_t i = 0U; i < IO_JOY_AI_COUNT; i++)
        {
            ai_copy[i] = ai_[i];
        }
        di_copy = di_mask_;
    }

    const float dz = static_cast<float>(deadzone_);
    for (uint8_t i = 0U; i < IO_JOY_AI_COUNT; i++)
    {
        joy_msg.axes[i] = static_cast<double>(apply_deadzone(ai_copy[i], dz));
    }

    for (uint8_t j = 0U; j < IO_JOY_DI_COUNT; j++)
    {
        joy_msg.buttons[j] = ((di_copy >> j) & 1U) ? 1 : 0;
    }

    joy_publisher_->publish(joy_msg);
}

void IoJoyBridge::timer_callback()
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
                "SEALIEN_IO_INPUT timeout (%d ms), stop publishing %s",
                link_timeout_ms_,
                joy_topic_.c_str());
            link_was_ok_ = false;
        }
        return;
    }

    if (!link_was_ok_)
    {
        link_was_ok_ = true;
    }

    publish_joy();
}
