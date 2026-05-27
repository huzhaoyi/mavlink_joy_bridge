/***** (C) Copyright, Sealien Robotics(Guangzhou) Co.,Ltd. ******source file****/

#include "sealien_mavlink_joy_bridge/mavlink_adc_bridge_core.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "mavlink.h"
#include "sealien_mavlink_joy_bridge/mavlink_rc_decode.hpp"

#define MADC_MAVLINK_COMM (0U)
#define MADC_UDP_BUF_SIZE (2048)
#define MADC_PUBLISH_HZ (50.0)

static mavlink_status_t g_mavlink_status;
static mavlink_message_t g_mavlink_message;

MavlinkAdcBridge::MavlinkAdcBridge(const std::string & node_name)
: Node(node_name),
  listen_port_(14550),
  link_timeout_ms_(500),
  udp_socket_fd_(-1),
  thread_running_(false),
  link_was_ok_(false),
  has_latest_(false)
{
    last_rx_time_ = std::chrono::steady_clock::now();
    param_init();

    adc_publisher_ = this->create_publisher<sealien_mavlink_joy_bridge::msg::AdcRaw>(
        "adc/raw", 10);

    thread_running_.store(true);
    udp_thread_ = std::thread(&MavlinkAdcBridge::udp_thread_func, this);

    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int64_t>(1000.0 / MADC_PUBLISH_HZ)),
        std::bind(&MavlinkAdcBridge::timer_callback, this));

    RCLCPP_INFO(
        this->get_logger(),
        "mavlink_adc_bridge: listen %s:%d, publish adc/raw, input %s",
        listen_address_.c_str(),
        listen_port_,
        mavlink_input_type_.c_str());
}

MavlinkAdcBridge::~MavlinkAdcBridge()
{
    thread_running_.store(false);
    close_udp_socket();
    if (udp_thread_.joinable())
    {
        udp_thread_.join();
    }
}

void MavlinkAdcBridge::param_init()
{
    listen_address_ = this->declare_parameter<std::string>("listen_address", "0.0.0.0");
    listen_port_ = this->declare_parameter<int>("listen_port", 14550);
    remote_ip_filter_ = this->declare_parameter<std::string>("remote_ip_filter", "");
    mavlink_input_type_ = this->declare_parameter<std::string>("mavlink_input_type", "auto");
    link_timeout_ms_ = this->declare_parameter<int>("link_timeout_ms", 500);
}

void MavlinkAdcBridge::close_udp_socket()
{
    if (udp_socket_fd_ >= 0)
    {
        close(udp_socket_fd_);
        udp_socket_fd_ = -1;
    }
}

void MavlinkAdcBridge::publish_adc_raw(
    uint8_t source_type,
    const uint16_t * channels,
    uint8_t channel_count,
    int8_t mode)
{
    sealien_mavlink_joy_bridge::msg::AdcRaw msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "adc";
    msg.source_type = source_type;
    msg.link_ok = 1U;
    msg.mode = mode;
    msg.channels.assign(channels, channels + channel_count);

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_adc_ = msg;
        has_latest_ = true;
        last_rx_time_ = std::chrono::steady_clock::now();
    }
}

void MavlinkAdcBridge::udp_thread_func()
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
        RCLCPP_ERROR(this->get_logger(), "bind failed: %s", strerror(errno));
        close_udp_socket();
        return;
    }

    struct timeval tv {};
    tv.tv_sec = 0;
    tv.tv_usec = 200000;
    setsockopt(udp_socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    uint8_t buf[MADC_UDP_BUF_SIZE];
    while (thread_running_.load() && rclcpp::ok())
    {
        struct sockaddr_in peer {};
        socklen_t peer_len = sizeof(peer);
        const ssize_t recv_len = recvfrom(
            udp_socket_fd_,
            buf,
            sizeof(buf),
            0,
            reinterpret_cast<struct sockaddr *>(&peer),
            &peer_len);

        if (recv_len < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
            {
                continue;
            }
            if (!thread_running_.load())
            {
                break;
            }
            continue;
        }

        if (!remote_ip_filter_.empty())
        {
            char peer_ip[INET_ADDRSTRLEN] = {0};
            inet_ntop(AF_INET, &peer.sin_addr, peer_ip, sizeof(peer_ip));
            if (remote_ip_filter_ != peer_ip)
            {
                continue;
            }
        }

        for (ssize_t i = 0; i < recv_len; i++)
        {
            if (mavlink_parse_char(
                    MADC_MAVLINK_COMM,
                    buf[i],
                    &g_mavlink_message,
                    &g_mavlink_status) != MAVLINK_FRAMING_OK)
            {
                continue;
            }

            uint16_t channels[ADC_RAW_MAX_CHANNELS] = {0U};
            uint8_t channel_count = 0U;
            uint8_t source_type = ADC_SRC_PWM_US;
            int8_t mode_val = 127;
            bool decoded = false;

            const bool want_auto = (mavlink_input_type_ == "auto");
            const bool want_manual = want_auto || (mavlink_input_type_ == "manual_control");
            const bool want_rc_raw = want_auto || (mavlink_input_type_ == "rc_channels_raw");
            const bool want_rc = want_auto || (mavlink_input_type_ == "rc_channels");
            const bool want_sealien = want_auto || (mavlink_input_type_ == "sealien_rc");

            if (want_manual &&
                g_mavlink_message.msgid == MAVLINK_MSG_ID_MANUAL_CONTROL &&
                mavlink_decode_manual_control(&g_mavlink_message, channels, &channel_count))
            {
                source_type = ADC_SRC_MANUAL_CONTROL;
                decoded = true;
            }
            else if (want_rc_raw &&
                g_mavlink_message.msgid == MAVLINK_MSG_ID_RC_CHANNELS_RAW &&
                mavlink_decode_rc_channels_raw(&g_mavlink_message, channels, &channel_count))
            {
                source_type = ADC_SRC_PWM_US;
                decoded = true;
            }
            else if (want_rc &&
                g_mavlink_message.msgid == MAVLINK_MSG_ID_RC_CHANNELS &&
                mavlink_decode_rc_channels(&g_mavlink_message, channels, &channel_count))
            {
                source_type = ADC_SRC_PWM_US;
                decoded = true;
            }
            else if (want_sealien &&
                g_mavlink_message.msgid == MAVLINK_MSG_ID_SEALIEN_RC_INPUT)
            {
                mavlink_sealien_rc_input_t rc_input {};
                mavlink_msg_sealien_rc_input_decode(&g_mavlink_message, &rc_input);
                if (mavlink_decode_sealien_rc_input(&g_mavlink_message, channels, &channel_count))
                {
                    source_type = ADC_SRC_SEALIEN_RC;
                    mode_val = rc_input.mode;
                    decoded = true;
                }
            }

            if (!decoded)
            {
                continue;
            }

            publish_adc_raw(source_type, channels, channel_count, mode_val);

            if (!link_was_ok_)
            {
                link_was_ok_ = true;
                RCLCPP_INFO(this->get_logger(), "MAVLink ADC link restored (msgid=%u)", g_mavlink_message.msgid);
            }
        }
    }

    close_udp_socket();
}

void MavlinkAdcBridge::timer_callback()
{
    sealien_mavlink_joy_bridge::msg::AdcRaw msg;
    bool publish = false;
    bool link_ok = false;

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (has_latest_)
        {
            const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - last_rx_time_).count();
            link_ok = elapsed_ms <= static_cast<int64_t>(link_timeout_ms_);
            msg = latest_adc_;
            msg.link_ok = link_ok ? 1U : 0U;
            publish = true;
        }
    }

    if (!link_ok && link_was_ok_)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "MAVLink ADC link timeout (%d ms)",
            link_timeout_ms_);
        link_was_ok_ = false;
    }

    if (publish)
    {
        adc_publisher_->publish(msg);
    }
}
