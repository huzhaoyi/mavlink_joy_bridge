/***** (C) Copyright, Sealien Robotics(Guangzhou) Co.,Ltd. ******header file****/
/*
 * 模块: adc_to_joy_mapper
 * 职责: 订阅 /mixed_io/raw (MixedIoRaw), 对每路 ADC 做 joystick 级校准
 *       后发布 sensor_msgs/Joy.
 * 处理流水线 (axes): 标定 -> 双边死区 -> expo -> invert -> LPF;
 *                    断线/越界 -> 强制 0 并清 LPF 状态.
 * 上游: mavlink_mixed_io_receiver (/mixed_io/raw)
 * 下游: 任意 sensor_msgs/Joy 消费者
 */
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sealien_mavlink_joy_bridge/msg/mixed_io_raw.hpp"
#include "sensor_msgs/msg/joy.hpp"

/* 三元 clamp; 仅用于内部数值钳位, 不做副作用 */
#define ADCJOY_LIMIT(x, min_val, max_val) \
    ((x) < (min_val) ? (min_val) : ((x) > (max_val) ? (max_val) : (x)))

/* ADC 通道 -> Joy.axes 索引映射项 */
typedef struct
{
    uint8_t adc_channel;
    uint8_t joy_axis;
} adcjoy_axis_map_t;

/* GPIO 位 -> Joy.buttons 索引映射项; active_high=false 表示低电平视为按下 */
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
    /* 声明并读取 ROS 参数, 仅在构造时调用一次 */
    void param_init();

    /* /mixed_io/raw 订阅回调: 链路 ok 则映射并发布, 否则直接 failsafe */
    void raw_callback(const sealien_mavlink_joy_bridge::msg::MixedIoRaw & msg);

    /* 发布 axes/buttons 全 0 的 Joy 并清 LPF 状态 */
    void publish_failsafe();

    /* 电压 -> Joy 单轴输出 [-1,1] 的完整流水线 (标定+死区+expo+invert), 不含 LPF */
    float map_axis(uint8_t ch_index, float voltage_v, float * out_raw_centered) const;

    /* 一阶低通: filtered = alpha*new + (1-alpha)*prev; alpha 按通道 clamp 到 [0,1] */
    float apply_filter(uint8_t joy_axis, uint8_t ch_index, float new_value);

    /* 断线/越界检测: NaN/Inf 或 V 超出 [adc_valid_min_v_, adc_valid_max_v_] 即为断线 */
    bool is_disconnected(float voltage_v) const;

    std::string raw_topic_;
    std::string joy_topic_;
    size_t axes_count_;
    size_t buttons_count_;

    /* 逐通道标定参数: 三段线性的端点与中位电压 (V) */
    std::vector<float> adc_min_v_;
    std::vector<float> adc_center_v_;
    std::vector<float> adc_max_v_;

    /* 双边死区作用在输出域 [-1,1], 不在电压域 */
    std::vector<double> deadzone_low_;
    std::vector<double> deadzone_high_;

    /* expo[i] ∈ [0,1]: 0=线性, 1=纯立方 */
    std::vector<double> expo_;
    std::vector<bool> invert_;
    std::vector<double> lpf_alpha_;

    /* 断线检测阈值 (V), 对所有通道统一 */
    double adc_valid_min_v_;
    double adc_valid_max_v_;

    std::vector<adcjoy_axis_map_t> axes_map_;
    std::vector<adcjoy_button_map_t> button_map_;

    /* LPF 上一帧输出, 按 joy_axis 索引 */
    std::vector<float> filtered_axes_;

    rclcpp::Subscription<sealien_mavlink_joy_bridge::msg::MixedIoRaw>::SharedPtr raw_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;
};
