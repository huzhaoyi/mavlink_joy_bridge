# sealien_mavlink_joy_bridge

ROS 2 包：接收机载 MCU 经 MAVLink **`MIXED_IO_DATA`**（msgid=20，与 sealien-ctrlcore 一致）上报的 **24 路 ADC 电压** 与 **GPIO 输入**，发布为 **`sensor_msgs/Joy`**。

## 数据流

```text
MCU (ADS7128 ×3 + TCA9535 ×4)
    │  MIXED_IO_DATA, UDP (默认 10 Hz)
    ▼
mixed_io_joy_bridge  ──/joy──► 下游

本机联调:
    网页 / mavlink_mixed_io_udp_send ──UDP 14550──► mixed_io_joy_bridge
```

## 编译

```bash
cd ~/sealien_ws
colcon build --packages-select sealien_mavlink_joy_bridge \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

## 启动

```bash
# 接 MCU
ros2 launch sealien_mavlink_joy_bridge sealien_mavlink_joy_bridge.launch.py

# 桥接 + 网页模拟（无需 MCU）
ros2 launch sealien_mavlink_joy_bridge sealien_mavlink_joy_bridge_sim.launch.py

# 仅网页（桥接另开终端）
ros2 run sealien_mavlink_joy_bridge start_rc_web_sim.sh
```

浏览器：`http://<主机IP>:8080/` — 24 路 ADC (0~5 V)、24 路 GPIO 输入。

## `/joy` 映射（与 MCU `task_comm.c` 一致）

**ADC**：`adc_dev1/2/3_Vin[8]` 展平为通道 0~23，线性映射  
`axis = (V - adc_min_v) / (adc_max_v - adc_min_v)`，钳位 **[0, 1]**（默认 0 V→0，5 V→1）。

**GPIO 输入**：`gpio_1_32_input` bit0~23 → `buttons[0~23]`（高有效，可配）。

默认 `axes_count` / `buttons_count` 均为 **24**，全通道一一映射。

## 配置

| 文件 | 说明 |
|------|------|
| `config/mixed_io_joy_bridge.yaml` | 桥接节点：端口、标定、通道映射 |
| `config/web_sim.yaml` | 网页 HTTP/UDP 目标 |

主要参数：`listen_port`（14550）、`link_timeout_ms`（500）、`adc_min_v` / `adc_max_v`、`axes_map_*`、`buttons_gpio_indices`。

## 验证

```bash
ros2 topic echo /joy
ros2 topic hz /joy

# 自动化（需已 install）
ros2 run sealien_mavlink_joy_bridge e2e_test.sh
```

## 命令行发包

```bash
ros2 run sealien_mavlink_joy_bridge mavlink_mixed_io_udp_send \
  <host> <port> <gpio_lo> <gpio_hi> <adc0_V> ... <adc23_V>
```

## MAVLink

- 方言头文件：`third_party/mavlink/sealien_mavlink/`（从 sealien-ctrlcore 同步）
- 本包仅使用 **`MIXED_IO_DATA`**（id 20）；下行 **`MIXED_IO_CMD`**（id 19）MCU 侧尚未实现接收

## 目录结构

```text
sealien_mavlink_joy_bridge/
├── config/
├── include/sealien_mavlink_joy_bridge/
├── launch/
├── scripts/          # rc_web_sim_server.py, start_rc_web_sim.sh, e2e_test.sh
├── src/              # mixed_io_joy_bridge
├── tools/            # mavlink_mixed_io_udp_send
├── web/
└── third_party/mavlink/sealien_mavlink/
```

## 许可证

Apache-2.0 — 维护者见 `package.xml`。
