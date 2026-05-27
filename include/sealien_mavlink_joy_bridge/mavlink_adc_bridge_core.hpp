#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sealien_mavlink_joy_bridge/msg/adc_raw.hpp"

class MavlinkAdcBridge : public rclcpp::Node
{
public:
    explicit MavlinkAdcBridge(const std::string & node_name);
    ~MavlinkAdcBridge() override;

private:
    void param_init();
    void udp_thread_func();
    void timer_callback();
    void publish_adc_raw(
        uint8_t source_type,
        const uint16_t * channels,
        uint8_t channel_count,
        int8_t mode);
    void close_udp_socket();

    std::string listen_address_;
    int listen_port_;
    std::string remote_ip_filter_;
    std::string mavlink_input_type_;
    int link_timeout_ms_;

    int udp_socket_fd_;
    std::atomic<bool> thread_running_;
    std::thread udp_thread_;

    std::mutex state_mutex_;
    std::chrono::steady_clock::time_point last_rx_time_;
    bool link_was_ok_;

    sealien_mavlink_joy_bridge::msg::AdcRaw latest_adc_;
    bool has_latest_;

    rclcpp::Publisher<sealien_mavlink_joy_bridge::msg::AdcRaw>::SharedPtr adc_publisher_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
};
