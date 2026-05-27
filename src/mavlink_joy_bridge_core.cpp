/***** (C) Copyright, Sealien Robotics(Guangzhou) Co.,Ltd. ******source file****
* File name          : mavlink_joy_bridge_core.cpp
* Author             : Sealien
* Brief              : MAVLink UDP to sensor_msgs/Joy bridge
********************************************************************************/

#include "sealien_mavlink_joy_bridge/mavlink_joy_bridge_core.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "mavlink.h"

#define MJOY_ZORRO_AXES_COUNT (7U)
#define MJOY_ZORRO_BUTTONS_COUNT (24U)
#define MJOY_MAVLINK_COMM (0U)
#define MJOY_UDP_RECV_BUF_SIZE (2048)
#define MJOY_MODE_AXES_INDEX (6U)

static mavlink_status_t g_mavlink_status;
static mavlink_message_t g_mavlink_message;

/********************************************************************************
 * @brief  :constructor
 *********************************************************************************/
MavlinkJoyBridge::MavlinkJoyBridge(const std::string & node_name)
: Node(node_name),
  listen_port_(14550),
  publish_rate_hz_(50.0),
  link_timeout_ms_(500),
  joy_topic_("/joy"),
  joy_profile_(MJOY_PROFILE_ZORRO),
  mode_from_mavlink_(true),
  mode_default_(0.0f),
  mode_channel_src_(7),
  axes_count_(MJOY_ZORRO_AXES_COUNT),
  buttons_count_(MJOY_ZORRO_BUTTONS_COUNT),
  udp_socket_fd_(-1),
  thread_running_(false),
  link_was_ok_(false)
{
    rc_state_.valid = false;
    rc_state_.seq = 0U;
    rc_state_.timestamp_ms = 0U;
    rc_state_.mode = 0;
    rc_state_.valid_mask = 0U;
    rc_state_.flags = 0U;
    for (uint8_t i = 0U; i < 16U; i++)
    {
        rc_state_.ch[i] = 0.0f;
    }

    last_rx_time_ = std::chrono::steady_clock::now();
    param_init();

    joy_publisher_ = this->create_publisher<sensor_msgs::msg::Joy>(joy_topic_, 10);

    thread_running_.store(true);
    udp_thread_ = std::thread(&MavlinkJoyBridge::udp_thread_func, this);

    const auto period_ms = static_cast<int64_t>(1000.0 / publish_rate_hz_);
    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_ms),
        std::bind(&MavlinkJoyBridge::timer_callback, this));

    RCLCPP_INFO(
        this->get_logger(),
        "mavlink_joy_bridge started: listen %s:%d, topic %s, profile zorro (%zu axes, %zu buttons)",
        listen_address_.c_str(),
        listen_port_,
        joy_topic_.c_str(),
        axes_count_,
        buttons_count_);
}

MavlinkJoyBridge::~MavlinkJoyBridge()
{
    thread_running_.store(false);
    close_udp_socket();
    if (udp_thread_.joinable())
    {
        udp_thread_.join();
    }
}

/********************************************************************************
 * @brief  :load parameters
 *********************************************************************************/
void MavlinkJoyBridge::param_init()
{
    listen_address_ = this->declare_parameter<std::string>("listen_address", "0.0.0.0");
    listen_port_ = this->declare_parameter<int>("listen_port", 14550);
    remote_ip_filter_ = this->declare_parameter<std::string>("remote_ip_filter", "");
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 50.0);
    link_timeout_ms_ = this->declare_parameter<int>("link_timeout_ms", 500);
    joy_topic_ = this->declare_parameter<std::string>("joy_topic", "/joy");
    mode_from_mavlink_ = this->declare_parameter<bool>("mode_from_mavlink", true);
    mode_default_ = static_cast<float>(this->declare_parameter<double>("mode_default", 0.0));
    mode_channel_src_ = this->declare_parameter<int>("mode_channel_src", 7);

    const std::string profile = this->declare_parameter<std::string>("joy_profile", "zorro");
    if (profile != "zorro")
    {
        RCLCPP_WARN(
            this->get_logger(),
            "joy_profile '%s' not supported, fallback to zorro",
            profile.c_str());
    }
    joy_profile_ = MJOY_PROFILE_ZORRO;
    axes_count_ = MJOY_ZORRO_AXES_COUNT;
    buttons_count_ = MJOY_ZORRO_BUTTONS_COUNT;

    mode_positions_ = this->declare_parameter<std::vector<double>>(
        "mode_positions",
        std::vector<double>{-1.0, 0.0, 1.0});

    const auto channel_map_strings = this->declare_parameter<std::vector<std::string>>(
        "channel_map",
        std::vector<std::string>{
            "0,axes,1,-1.0,0.05",
            "1,axes,0,1.0,0.05",
            "2,axes,2,-1.0,0.05",
            "3,axes,3,1.0,0.05"});

    channel_maps_.clear();
    for (const auto & entry : channel_map_strings)
    {
        mjoy_channel_map_t map_entry{};
        if (parse_channel_map_entry(entry, &map_entry))
        {
            channel_maps_.push_back(map_entry);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "invalid channel_map entry: '%s'", entry.c_str());
        }
    }

    if (channel_maps_.empty())
    {
        RCLCPP_WARN(this->get_logger(), "channel_map is empty, Joy axes/buttons will stay zero");
    }
}

/********************************************************************************
 * @brief  :parse one channel_map string: src,dst,index,scale,deadzone
 *********************************************************************************/
bool MavlinkJoyBridge::parse_channel_map_entry(const std::string & entry, mjoy_channel_map_t * out)
{
    if (out == nullptr)
    {
        return false;
    }

    std::string token;
    std::vector<std::string> parts;
    for (char c : entry)
    {
        if (c == ',')
        {
            parts.push_back(token);
            token.clear();
        }
        else
        {
            token.push_back(c);
        }
    }
    parts.push_back(token);

    if (parts.size() != 5U)
    {
        return false;
    }

    try
    {
        out->src = static_cast<uint8_t>(std::stoi(parts[0]));
        if (parts[1] == "axes")
        {
            out->dst_type = MJOY_DST_AXES;
        }
        else if (parts[1] == "buttons")
        {
            out->dst_type = MJOY_DST_BUTTONS;
        }
        else
        {
            return false;
        }
        out->index = static_cast<uint8_t>(std::stoi(parts[2]));
        out->scale = std::stof(parts[3]);
        out->deadzone = std::stof(parts[4]);
    }
    catch (const std::exception &)
    {
        return false;
    }

    if (out->src >= 16U)
    {
        return false;
    }

    return true;
}

/********************************************************************************
 * @brief  :close udp socket
 *********************************************************************************/
void MavlinkJoyBridge::close_udp_socket()
{
    if (udp_socket_fd_ >= 0)
    {
        close(udp_socket_fd_);
        udp_socket_fd_ = -1;
    }
}

/********************************************************************************
 * @brief  :udp receive thread
 *********************************************************************************/
void MavlinkJoyBridge::udp_thread_func()
{
    udp_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_socket_fd_ < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "socket() failed: %s", strerror(errno));
        return;
    }

    int reuse = 1;
    if (setsockopt(udp_socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0)
    {
        RCLCPP_WARN(this->get_logger(), "setsockopt SO_REUSEADDR failed: %s", strerror(errno));
    }

    struct sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(listen_port_));
    if (inet_pton(AF_INET, listen_address_.c_str(), &addr.sin_addr) <= 0)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "inet_pton failed for listen_address '%s'",
            listen_address_.c_str());
        close_udp_socket();
        return;
    }

    if (bind(udp_socket_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "bind %s:%d failed: %s",
            listen_address_.c_str(),
            listen_port_,
            strerror(errno));
        close_udp_socket();
        return;
    }

    struct timeval tv {};
    tv.tv_sec = 0;
    tv.tv_usec = 200000;
    if (setsockopt(udp_socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0)
    {
        RCLCPP_WARN(this->get_logger(), "setsockopt SO_RCVTIMEO failed: %s", strerror(errno));
    }

    RCLCPP_INFO(
        this->get_logger(),
        "UDP listening on %s:%d",
        listen_address_.c_str(),
        listen_port_);

    uint8_t buf[MJOY_UDP_RECV_BUF_SIZE];
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
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "recvfrom error: %s",
                strerror(errno));
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
                    MJOY_MAVLINK_COMM,
                    buf[i],
                    &g_mavlink_message,
                    &g_mavlink_status) != MAVLINK_FRAMING_OK)
            {
                continue;
            }

            if (g_mavlink_message.msgid != MAVLINK_MSG_ID_SEALIEN_RC_INPUT)
            {
                continue;
            }

            mavlink_sealien_rc_input_t rc_input {};
            mavlink_msg_sealien_rc_input_decode(&g_mavlink_message, &rc_input);

            mjoy_rc_state_t snapshot {};
            {
                std::lock_guard<std::mutex> lock(rc_mutex_);
                rc_state_.valid = true;
                rc_state_.seq = rc_input.seq;
                rc_state_.timestamp_ms = rc_input.timestamp_ms;
                rc_state_.mode = rc_input.mode;
                rc_state_.valid_mask = rc_input.valid_mask;
                rc_state_.flags = rc_input.flags;
                for (uint8_t ch_idx = 0U; ch_idx < 16U; ch_idx++)
                {
                    rc_state_.ch[ch_idx] = rc_input.ch[ch_idx];
                }
                last_rx_time_ = std::chrono::steady_clock::now();
                snapshot = rc_state_;
            }

            if (!link_was_ok_)
            {
                link_was_ok_ = true;
                RCLCPP_INFO(this->get_logger(), "MAVLink RC link restored");
            }
        }
    }

    close_udp_socket();
}

/********************************************************************************
 * @brief  :deadzone and scale
 *********************************************************************************/
float MavlinkJoyBridge::apply_axis_mapping(float raw, float scale, float deadzone) const
{
    float value = raw * scale;
    if (std::fabs(value) < deadzone)
    {
        value = 0.0f;
    }
    return MJOY_LIMIT(value, -1.0f, 1.0f);
}

/********************************************************************************
 * @brief  :map mode to zorro axes[6]
 *********************************************************************************/
float MavlinkJoyBridge::apply_mode_value(const mjoy_rc_state_t & rc) const
{
    if (mode_from_mavlink_)
    {
        const int8_t mode_val = rc.mode;
        if (mode_val <= -1)
        {
            return -1.0f;
        }
        if (mode_val >= 1)
        {
            return 1.0f;
        }
        return 0.0f;
    }

    if (mode_channel_src_ >= 0 && mode_channel_src_ < 16)
    {
        const float raw = rc.ch[static_cast<uint8_t>(mode_channel_src_)];
        if (mode_positions_.size() >= 3U)
        {
            const float low = static_cast<float>(mode_positions_[0]);
            const float mid = static_cast<float>(mode_positions_[1]);
            const float high = static_cast<float>(mode_positions_[2]);
            if (raw < -0.33f)
            {
                return low;
            }
            if (raw > 0.33f)
            {
                return high;
            }
            return mid;
        }
    }

    return MJOY_LIMIT(mode_default_, -1.0f, 1.0f);
}

/********************************************************************************
 * @brief  :apply configured channel map
 *********************************************************************************/
void MavlinkJoyBridge::apply_channel_maps(const mjoy_rc_state_t & rc, sensor_msgs::msg::Joy * joy_msg)
{
    if (joy_msg == nullptr)
    {
        return;
    }

    for (const auto & entry : channel_maps_)
    {
        const float raw = rc.ch[entry.src];
        if (entry.dst_type == MJOY_DST_AXES)
        {
            if (entry.index < joy_msg->axes.size())
            {
                joy_msg->axes[entry.index] = apply_axis_mapping(raw, entry.scale, entry.deadzone);
            }
        }
        else if (entry.dst_type == MJOY_DST_BUTTONS)
        {
            if (entry.index < joy_msg->buttons.size())
            {
                const float mapped = apply_axis_mapping(raw, entry.scale, entry.deadzone);
                joy_msg->buttons[entry.index] = (mapped > 0.5f) ? 1 : 0;
            }
        }
    }

    if (MJOY_MODE_AXES_INDEX < joy_msg->axes.size())
    {
        joy_msg->axes[MJOY_MODE_AXES_INDEX] = apply_mode_value(rc);
    }
}

/********************************************************************************
 * @brief  :build and publish Joy from RC snapshot
 *********************************************************************************/
void MavlinkJoyBridge::publish_joy_from_rc(const mjoy_rc_state_t & rc)
{
    if (!rc.valid)
    {
        return;
    }

    sensor_msgs::msg::Joy joy_msg;
    joy_msg.header.stamp = this->now();
    joy_msg.header.frame_id = "zorro";
    joy_msg.axes.assign(axes_count_, 0.0);
    joy_msg.buttons.assign(buttons_count_, 0);
    apply_channel_maps(rc, &joy_msg);
    joy_publisher_->publish(joy_msg);
}

/********************************************************************************
 * @brief  :watchdog timer (link timeout)
 *********************************************************************************/
void MavlinkJoyBridge::timer_callback()
{
    mjoy_rc_state_t snapshot {};
    bool link_ok = false;

    {
        std::lock_guard<std::mutex> lock(rc_mutex_);
        const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - last_rx_time_).count();
        link_ok = rc_state_.valid && (elapsed_ms <= static_cast<int64_t>(link_timeout_ms_));
        if (link_ok)
        {
            snapshot = rc_state_;
        }
    }

    if (!link_ok)
    {
        if (link_was_ok_)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "MAVLink RC link timeout (%d ms), stop publishing %s",
                link_timeout_ms_,
                joy_topic_.c_str());
            link_was_ok_ = false;
        }
        return;
    }

    if (!link_was_ok_)
    {
        link_was_ok_ = true;
        RCLCPP_INFO(this->get_logger(), "MAVLink RC link restored");
    }

    publish_joy_from_rc(snapshot);
}
