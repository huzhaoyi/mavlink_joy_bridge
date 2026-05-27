#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sealien_mavlink_joy_bridge/msg/mixed_io_raw.hpp"
#include "sensor_msgs/msg/joy.hpp"

#define ADCJOY_LIMIT(x, min_val, max_val) \
    ((x) < (min_val) ? (min_val) : ((x) > (max_val) ? (max_val) : (x)))

typedef struct
{
    uint8_t adc_channel;
    uint8_t joy_axis;
} adcjoy_axis_map_t;

typedef struct
{
    uint8_t gpio_index;
    uint8_t joy_button;
    bool active_high;
} adcjoy_button_map_t;

class AdcToJoyMapper : public rclcpp::Node
{
public:
    explicit AdcToJoyMapper(const std::string & node_name);
    ~AdcToJoyMapper() override = default;

private:
    void param_init();
    void raw_callback(const sealien_mavlink_joy_bridge::msg::MixedIoRaw & msg);
    void publish_failsafe();
    float map_axis(uint8_t ch_index, float voltage_v, float * out_raw_centered) const;
    float apply_filter(uint8_t joy_axis, uint8_t ch_index, float new_value);
    bool is_disconnected(float voltage_v) const;

    std::string raw_topic_;
    std::string joy_topic_;
    size_t axes_count_;
    size_t buttons_count_;

    std::vector<float> adc_min_v_;
    std::vector<float> adc_center_v_;
    std::vector<float> adc_max_v_;
    std::vector<double> deadzone_low_;
    std::vector<double> deadzone_high_;
    std::vector<double> expo_;
    std::vector<bool> invert_;
    std::vector<double> lpf_alpha_;
    double adc_valid_min_v_;
    double adc_valid_max_v_;

    std::vector<adcjoy_axis_map_t> axes_map_;
    std::vector<adcjoy_button_map_t> button_map_;

    std::vector<float> filtered_axes_;

    rclcpp::Subscription<sealien_mavlink_joy_bridge::msg::MixedIoRaw>::SharedPtr raw_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;
};
