#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sealien_mavlink_joy_bridge/msg/adc_raw.hpp"
#include "sensor_msgs/msg/joy.hpp"

#define AJOY_LIMIT(x, min_val, max_val) ((x) < (min_val) ? (min_val) : ((x) > (max_val) ? (max_val) : (x)))

typedef struct
{
    uint8_t adc_channel;
    uint8_t joy_axis;
} ajoy_axis_map_t;

typedef struct
{
    uint8_t adc_channel;
    uint8_t joy_button;
    uint16_t threshold;
} ajoy_button_map_t;

class AdcJoyMapper : public rclcpp::Node
{
public:
    explicit AdcJoyMapper(const std::string & node_name);
    ~AdcJoyMapper() override = default;

private:
    void param_init();
    void adc_callback(const sealien_mavlink_joy_bridge::msg::AdcRaw & msg);
    void timer_callback();
    float normalize_channel(uint8_t ch_index, uint16_t raw_us) const;
    float apply_mode_from_adc(uint16_t raw_us) const;
    void publish_joy();

    std::string adc_topic_;
    std::string joy_topic_;
    double publish_rate_hz_;
    int link_timeout_ms_;

    std::vector<uint16_t> adc_min_;
    std::vector<uint16_t> adc_max_;
    std::vector<uint16_t> adc_center_;
    std::vector<double> deadzone_;
    std::vector<bool> invert_;
    std::vector<double> lpf_alpha_;
    std::vector<ajoy_axis_map_t> axes_map_;
    std::vector<ajoy_button_map_t> button_map_;
    int mode_adc_channel_;
    std::vector<double> mode_positions_;

    size_t axes_count_;
    size_t buttons_count_;

    sealien_mavlink_joy_bridge::msg::AdcRaw latest_adc_;
    bool has_adc_;
    std::vector<float> filtered_axes_;
    std::chrono::steady_clock::time_point last_adc_time_;
    bool link_was_ok_;

    rclcpp::Subscription<sealien_mavlink_joy_bridge::msg::AdcRaw>::SharedPtr adc_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
};
