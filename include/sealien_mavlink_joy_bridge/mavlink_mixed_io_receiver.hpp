/***** (C) Copyright, Sealien Robotics(Guangzhou) Co.,Ltd. ******header file****/
/*
 * 模块: mavlink_mixed_io_receiver
 * 职责: 监听 UDP 上的 MAVLink MIXED_IO_DATA, 解析后以事件驱动方式发布
 *       /mixed_io/raw (MixedIoRaw); 链路超时持续发 link_ok=false 帧.
 * 上游: MCU / 网页模拟器 (UDP)
 * 下游: adc_to_joy_mapper (订阅 /mixed_io/raw)
 */
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sealien_mavlink_joy_bridge/msg/mixed_io_raw.hpp"

/* 与 MCU MIXED_IO_DATA 一致: 3×ADS7128, 每片 8 通道 = 24 路 */
#define MIXED_IO_RECV_ADC_COUNT (24U)

class MavlinkMixedIoReceiver : public rclcpp::Node
{
public:
    explicit MavlinkMixedIoReceiver(const std::string & node_name);
    ~MavlinkMixedIoReceiver() override;

private:
    /* 声明并读取 ROS 参数, 仅在构造时调用一次 */
    void param_init();

    /* UDP 接收线程主循环: 阻塞 recvfrom -> mavlink_parse_char -> 立即发布 */
    void udp_thread_func();

    /* failsafe 定时器: 周期检查 last_rx_time_, 超时则发 link_ok=false 帧 */
    void failsafe_timer_callback();

    /* 关闭 UDP socket, 幂等 */
    void close_udp_socket();

    /* 组装并发布 /mixed_io/raw; link_ok=false 时调用方应传零数据 */
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

    /* 跨线程共享状态: UDP 线程写, 定时器线程读 */
    std::mutex state_mutex_;
    std::chrono::steady_clock::time_point last_rx_time_;
    bool has_rx_;
    bool link_was_ok_;

    rclcpp::Publisher<sealien_mavlink_joy_bridge::msg::MixedIoRaw>::SharedPtr raw_publisher_;
    rclcpp::TimerBase::SharedPtr failsafe_timer_;
};
