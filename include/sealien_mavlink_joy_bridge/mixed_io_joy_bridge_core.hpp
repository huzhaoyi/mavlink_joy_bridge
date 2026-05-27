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

#define MIXED_IO_ADC_COUNT (24U)
#define MIXED_IO_GPIO_COUNT (64U)

#define MIXED_IO_LIMIT(x, min_val, max_val) \
    ((x) < (min_val) ? (min_val) : ((x) > (max_val) ? (max_val) : (x)))

typedef struct
{
    uint8_t adc_channel;
    uint8_t joy_axis;
} mixed_io_axis_map_t;

typedef struct
{
    uint8_t gpio_index;
    uint8_t joy_button;
    bool active_high;
} mixed_io_button_map_t;

class MixedIoJoyBridge : public rclcpp::Node
{
public:
    explicit MixedIoJoyBridge(const std::string & node_name);
    ~MixedIoJoyBridge() override;

private:
    void param_init();
    void udp_thread_func();
    void timer_callback();
    void publish_joy();
    void close_udp_socket();
    float normalize_voltage(uint8_t ch_index, float voltage_v) const;

    std::string listen_address_;
    int listen_port_;
    std::string remote_ip_filter_;
    std::string joy_topic_;
    double publish_rate_hz_;
    int link_timeout_ms_;
    size_t axes_count_;
    size_t buttons_count_;

    std::vector<float> adc_min_v_;
    std::vector<float> adc_max_v_;
    std::vector<double> deadzone_;
    std::vector<bool> invert_;
    std::vector<double> lpf_alpha_;
    std::vector<mixed_io_axis_map_t> axes_map_;
    std::vector<mixed_io_button_map_t> button_map_;

    int udp_socket_fd_;
    std::atomic<bool> thread_running_;
    std::thread udp_thread_;

    std::mutex state_mutex_;
    std::chrono::steady_clock::time_point last_rx_time_;
    bool link_was_ok_;
    bool has_latest_;

    float adc_v_[MIXED_IO_ADC_COUNT];
    uint64_t gpio_input_mask_;
    std::vector<float> filtered_axes_;

    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
};
