#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#define IO_JOY_AI_COUNT (24U)
#define IO_JOY_DI_COUNT (24U)

class IoJoyBridge : public rclcpp::Node
{
public:
    explicit IoJoyBridge(const std::string & node_name);
    ~IoJoyBridge() override;

private:
    void param_init();
    void udp_thread_func();
    void timer_callback();
    void publish_joy();
    void close_udp_socket();

    std::string listen_address_;
    int listen_port_;
    std::string remote_ip_filter_;
    std::string joy_topic_;
    double publish_rate_hz_;
    int link_timeout_ms_;
    double deadzone_;

    int udp_socket_fd_;
    std::atomic<bool> thread_running_;
    std::thread udp_thread_;

    std::mutex state_mutex_;
    std::chrono::steady_clock::time_point last_rx_time_;
    bool link_was_ok_;
    bool has_latest_;

    float ai_[IO_JOY_AI_COUNT];
    uint32_t di_mask_;

    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
};
