#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sealien_mavlink_joy_bridge/msg/mixed_io_raw.hpp"

#define MIXED_IO_RECV_ADC_COUNT (24U)

class MavlinkMixedIoReceiver : public rclcpp::Node
{
public:
    explicit MavlinkMixedIoReceiver(const std::string & node_name);
    ~MavlinkMixedIoReceiver() override;

private:
    void param_init();
    void udp_thread_func();
    void failsafe_timer_callback();
    void close_udp_socket();
    void publish_raw(bool link_ok,
                     uint32_t mcu_timestamp_ms,
                     const float adc_v[MIXED_IO_RECV_ADC_COUNT],
                     uint64_t gpio_input_mask);

    std::string listen_address_;
    int listen_port_;
    std::string remote_ip_filter_;
    std::string raw_topic_;
    int link_timeout_ms_;
    double failsafe_publish_rate_hz_;

    int udp_socket_fd_;
    std::atomic<bool> thread_running_;
    std::thread udp_thread_;

    std::mutex state_mutex_;
    std::chrono::steady_clock::time_point last_rx_time_;
    bool has_rx_;
    bool link_was_ok_;

    rclcpp::Publisher<sealien_mavlink_joy_bridge::msg::MixedIoRaw>::SharedPtr raw_publisher_;
    rclcpp::TimerBase::SharedPtr failsafe_timer_;
};
