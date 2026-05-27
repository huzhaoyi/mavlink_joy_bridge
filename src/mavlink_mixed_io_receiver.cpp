/***** (C) Copyright, Sealien Robotics(Guangzhou) Co.,Ltd. ******source file****/
/*
 * mavlink_mixed_io_receiver 实现.
 * 线程模型:
 *   - UDP 线程: 阻塞读取 + MAVLink 解析 + 立即发布 (事件驱动)
 *   - ROS 定时器线程: 50Hz 检查超时, 仅在超时时发 failsafe
 * 共享状态由 state_mutex_ 保护.
 */

#include "sealien_mavlink_joy_bridge/mavlink_mixed_io_receiver.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "sealien_mavlink/mavlink.h"

#define MIXED_IO_RECV_MAVLINK_COMM (0U)
#define MIXED_IO_RECV_UDP_BUF_SIZE (2048)

/* MAVLink 解析状态机: 单 channel, 全局静态, 仅 UDP 线程访问 */
static mavlink_status_t g_mavlink_status;
static mavlink_message_t g_mavlink_message;

/* 将 MCU 两个 uint32 GPIO 输入位图拼成 uint64; bit0 对应 GPIO0 */
static uint64_t pack_gpio_input_mask(uint32_t lo, uint32_t hi)
{
    return (static_cast<uint64_t>(hi) << 32) | static_cast<uint64_t>(lo);
}

/* 构造: 声明参数 -> 建发布器 -> 启动 UDP 线程 -> 启动 failsafe 定时器 */
MavlinkMixedIoReceiver::MavlinkMixedIoReceiver(const std::string & node_name)
: Node(node_name),
  listen_port_(14550),
  raw_topic_("/mixed_io/raw"),
  link_timeout_ms_(200),
  failsafe_publish_rate_hz_(50.0),
  udp_socket_fd_(-1),
  thread_running_(false),
  has_rx_(false),
  link_was_ok_(false)
{
    last_rx_time_ = std::chrono::steady_clock::now();
    param_init();

    raw_publisher_ = this->create_publisher<sealien_mavlink_joy_bridge::msg::MixedIoRaw>(
        raw_topic_, 10);

    thread_running_.store(true);
    udp_thread_ = std::thread(&MavlinkMixedIoReceiver::udp_thread_func, this);

    const auto period_ms = static_cast<int64_t>(1000.0 /
        (failsafe_publish_rate_hz_ > 0.1 ? failsafe_publish_rate_hz_ : 50.0));
    failsafe_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_ms),
        std::bind(&MavlinkMixedIoReceiver::failsafe_timer_callback, this));

    RCLCPP_INFO(
        this->get_logger(),
        "mavlink_mixed_io_receiver: listen %s:%d, publish %s (timeout %d ms, failsafe %.1f Hz)",
        listen_address_.c_str(),
        listen_port_,
        raw_topic_.c_str(),
        link_timeout_ms_,
        failsafe_publish_rate_hz_);
}

/* 析构: 关 socket 中断阻塞 recvfrom, 等待线程退出 */
MavlinkMixedIoReceiver::~MavlinkMixedIoReceiver()
{
    thread_running_.store(false);
    close_udp_socket();
    if (udp_thread_.joinable())
    {
        udp_thread_.join();
    }
}

/* 声明并读取 ROS 参数, 默认值与 YAML 一致 */
void MavlinkMixedIoReceiver::param_init()
{
    listen_address_ = this->declare_parameter<std::string>("listen_address", "0.0.0.0");
    listen_port_ = this->declare_parameter<int>("listen_port", 14550);
    remote_ip_filter_ = this->declare_parameter<std::string>("remote_ip_filter", "");
    raw_topic_ = this->declare_parameter<std::string>("raw_topic", "/mixed_io/raw");
    link_timeout_ms_ = this->declare_parameter<int>("link_timeout_ms", 200);
    failsafe_publish_rate_hz_ = this->declare_parameter<double>("failsafe_publish_rate_hz", 50.0);
}

/* 幂等关闭 UDP socket: 析构与 bind 失败路径都会调 */
void MavlinkMixedIoReceiver::close_udp_socket()
{
    if (udp_socket_fd_ >= 0)
    {
        close(udp_socket_fd_);
        udp_socket_fd_ = -1;
    }
}

/* 组装并发布一条 /mixed_io/raw; 调用方负责保证 adc_v 至少 24 个元素 */
void MavlinkMixedIoReceiver::publish_raw(
    bool link_ok,
    uint32_t mcu_timestamp_ms,
    const float adc_v[MIXED_IO_RECV_ADC_COUNT],
    uint64_t gpio_input_mask)
{
    sealien_mavlink_joy_bridge::msg::MixedIoRaw msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = link_ok ? "mixed_io" : "mixed_io_failsafe";
    msg.mcu_timestamp_ms = mcu_timestamp_ms;
    msg.link_ok = link_ok;
    msg.adc_v.assign(adc_v, adc_v + MIXED_IO_RECV_ADC_COUNT);
    msg.gpio_input_mask = gpio_input_mask;
    raw_publisher_->publish(msg);
}

/* UDP 线程主循环
 * 1) 建 socket 并 bind; 失败直接返回 (节点保持运行, 定时器会持续发 failsafe)
 * 2) 阻塞 recvfrom, 收到一包后按可选 IP 白名单过滤
 * 3) 逐字节喂 mavlink_parse_char; 只关心 MIXED_IO_DATA(20)
 * 4) 展平 24 路 ADC + 拼 64 位 GPIO mask, 更新 last_rx_time_, 立即发布
 */
void MavlinkMixedIoReceiver::udp_thread_func()
{
    udp_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_socket_fd_ < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "socket() failed: %s", strerror(errno));
        return;
    }

    int reuse = 1;
    setsockopt(udp_socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(listen_port_));
    if (inet_pton(AF_INET, listen_address_.c_str(), &addr.sin_addr) <= 0)
    {
        RCLCPP_ERROR(this->get_logger(), "invalid listen_address");
        close_udp_socket();
        return;
    }

    if (bind(udp_socket_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "bind() failed: %s", strerror(errno));
        close_udp_socket();
        return;
    }

    uint8_t buf[MIXED_IO_RECV_UDP_BUF_SIZE];
    while (thread_running_.load())
    {
        struct sockaddr_in from_addr {};
        socklen_t from_len = sizeof(from_addr);
        const ssize_t n = recvfrom(
            udp_socket_fd_,
            buf,
            sizeof(buf),
            0,
            reinterpret_cast<struct sockaddr *>(&from_addr),
            &from_len);

        if (n < 0)
        {
            // 析构关 socket 会让阻塞 recvfrom 返回错误, 用 thread_running_ 区分
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
                2000,
                "recvfrom: %s",
                strerror(errno));
            continue;
        }

        // 可选 IP 白名单: 空串表示不过滤
        if (!remote_ip_filter_.empty())
        {
            char ip_str[INET_ADDRSTRLEN] = {0};
            inet_ntop(AF_INET, &from_addr.sin_addr, ip_str, sizeof(ip_str));
            if (remote_ip_filter_ != ip_str)
            {
                continue;
            }
        }

        for (ssize_t i = 0; i < n; i++)
        {
            if (mavlink_parse_char(
                    MIXED_IO_RECV_MAVLINK_COMM,
                    buf[i],
                    &g_mavlink_message,
                    &g_mavlink_status) != MAVLINK_FRAMING_OK)
            {
                continue;
            }

            if (g_mavlink_message.msgid != MAVLINK_MSG_ID_MIXED_IO_DATA)
            {
                continue;
            }

            mavlink_mixed_io_data_t io_data {};
            mavlink_msg_mixed_io_data_decode(&g_mavlink_message, &io_data);

            // 3×8 ADC 展平为 [dev1_ch0..7, dev2_ch0..7, dev3_ch0..7]
            float adc_v[MIXED_IO_RECV_ADC_COUNT] = {0.0f};
            for (uint8_t k = 0U; k < 8U; k++)
            {
                adc_v[k] = io_data.adc_dev1_Vin[k];
                adc_v[8U + k] = io_data.adc_dev2_Vin[k];
                adc_v[16U + k] = io_data.adc_dev3_Vin[k];
            }
            const uint64_t gpio_mask = pack_gpio_input_mask(
                io_data.gpio_1_32_input,
                io_data.gpio_33_64_input);

            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                has_rx_ = true;
                last_rx_time_ = std::chrono::steady_clock::now();
                if (!link_was_ok_)
                {
                    link_was_ok_ = true;
                    RCLCPP_INFO(this->get_logger(), "MIXED_IO_DATA link active");
                }
            }

            publish_raw(true, io_data.timestamp_ms, adc_v, gpio_mask);
        }
    }
}

/* failsafe 定时器: 链路活时不发 (避免与 UDP 线程重复发布), 仅超时时持续发零帧 */
void MavlinkMixedIoReceiver::failsafe_timer_callback()
{
    bool need_failsafe = false;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - last_rx_time_).count();
        const bool link_ok = has_rx_ &&
            (elapsed_ms <= static_cast<int64_t>(link_timeout_ms_));

        if (!link_ok)
        {
            if (link_was_ok_)
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "MIXED_IO_DATA timeout (%d ms), publishing failsafe %s",
                    link_timeout_ms_,
                    raw_topic_.c_str());
                link_was_ok_ = false;
            }
            need_failsafe = true;
        }
    }

    if (need_failsafe)
    {
        float zero_v[MIXED_IO_RECV_ADC_COUNT] = {0.0f};
        publish_raw(false, 0U, zero_v, 0U);
    }
}
