#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#define MJOY_LIMIT(x, min_val, max_val) ((x) < (min_val) ? (min_val) : ((x) > (max_val) ? (max_val) : (x)))

typedef enum
{
    MJOY_DST_AXES = 0,
    MJOY_DST_BUTTONS = 1,
} mjoy_dst_type_e;

typedef enum
{
    MJOY_PROFILE_ZORRO = 0,
} mjoy_profile_e;

typedef struct
{
    uint8_t src;
    mjoy_dst_type_e dst_type;
    uint8_t index;
    float scale;
    float deadzone;
} mjoy_channel_map_t;

typedef struct
{
    bool valid;
    uint32_t seq;
    uint32_t timestamp_ms;
    float ch[16];
    int8_t mode;
    uint8_t valid_mask;
    uint8_t flags;
} mjoy_rc_state_t;

class MavlinkJoyBridge : public rclcpp::Node
{
public:
    explicit MavlinkJoyBridge(const std::string & node_name);
    ~MavlinkJoyBridge() override;

private:
    void param_init();
    bool parse_channel_map_entry(const std::string & entry, mjoy_channel_map_t * out);
    void udp_thread_func();
    void timer_callback();
    void apply_channel_maps(const mjoy_rc_state_t & rc, sensor_msgs::msg::Joy * joy_msg);
    void publish_joy_from_rc(const mjoy_rc_state_t & rc);
    float apply_axis_mapping(float raw, float scale, float deadzone) const;
    float apply_mode_value(const mjoy_rc_state_t & rc) const;
    void close_udp_socket();

    std::string listen_address_;
    int listen_port_;
    std::string remote_ip_filter_;
    double publish_rate_hz_;
    int link_timeout_ms_;
    std::string joy_topic_;
    mjoy_profile_e joy_profile_;
    bool mode_from_mavlink_;
    float mode_default_;
    int mode_channel_src_;
    std::vector<double> mode_positions_;

    std::vector<mjoy_channel_map_t> channel_maps_;

    size_t axes_count_;
    size_t buttons_count_;

    int udp_socket_fd_;
    std::atomic<bool> thread_running_;
    std::thread udp_thread_;

    std::mutex rc_mutex_;
    mjoy_rc_state_t rc_state_;
    std::chrono::steady_clock::time_point last_rx_time_;
    bool link_was_ok_;

    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
};
