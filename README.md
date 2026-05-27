# sealien_mavlink_joy_bridge

ROS 2 包：将机载 MCU 采集的 **24 路模拟量 (AI)** 与 **24 路数字量 (DI)**，经 **MAVLink `SEALIEN_IO_INPUT`**（UDP）桥接为 **`sensor_msgs/Joy`**，供下游遥控/任务节点使用。

## 功能概览

| 方向 | 内容 |
|------|------|
| 输入 | UDP 上的 MAVLink 帧，消息 ID **203** `SEALIEN_IO_INPUT` |
| 输出 | `sensor_msgs/Joy`：`axes[0..23]` ← AI，`buttons[0..23]` ← DI |
| 特点 | 单帧 24+24 路，**无 mode 轴**；固定 50 Hz 发布（可配）；链路超时停止发布 |

## 架构

### 当前推荐（`io_joy_bridge`）

```text
MCU / 外设
    │  SEALIEN_IO_INPUT (MAVLink, UDP)
    ▼
io_joy_bridge  ──/joy (sensor_msgs/Joy)──► 下游 (onboardcontrol 等)

开发/测试:
    网页模拟器 ──UDP──► io_joy_bridge
    mavlink_io_udp_send ──UDP──► io_joy_bridge
```

### 遗留两阶段链路（仍编译，默认 launch 未启用）

```text
MAVLink (多种 RC/ADC 报文)
    ▼
mavlink_adc_bridge  ──/adc/raw (AdcRaw)──►
    ▼
adc_joy_mapper  ──/joy──► 下游

或单节点:

mavlink_joy_bridge  ──/joy──►  (SEALIEN_RC_INPUT + channel_map + mode)
```

## 依赖

- ROS 2（`rclcpp`、`sensor_msgs`、`std_msgs`）
- `colcon` / `ament_cmake`
- 网页模拟：`python3-yaml`（`apt install python3-yaml` 或已通过 `package.xml` 声明）

## 编译

```bash
cd ~/sealien_ws
colcon build --packages-select sealien_mavlink_joy_bridge \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

## 可执行文件与节点

| 可执行文件 | 说明 | 默认 launch |
|------------|------|-------------|
| `io_joy_bridge` | **主节点**：UDP → `/joy` | 是 |
| `mavlink_io_udp_send` | 命令行发送 `SEALIEN_IO_INPUT` | — |
| `rc_web_sim_server.py` | HTTP 网页 + 定时 UDP 发包 | sim launch |
| `mavlink_adc_bridge` | 遗留：MAVLink → `AdcRaw` | 否 |
| `adc_joy_mapper` | 遗留：`AdcRaw` → `/joy` | 否 |
| `mavlink_joy_bridge` | 遗留：RC + 映射 → `/joy` | 否 |
| `mavlink_rc_udp_send` | 遗留：发送 `SEALIEN_RC_INPUT` | — |

## 启动

```bash
# 仅桥接（接 MCU 或外部 UDP 源）
ros2 launch sealien_mavlink_joy_bridge sealien_mavlink_joy_bridge.launch.py

# 桥接 + 网页模拟（本机联调）
ros2 launch sealien_mavlink_joy_bridge sealien_mavlink_joy_bridge_sim.launch.py

# 单独启动网页（需已 source install；桥接需另开终端）
ros2 run sealien_mavlink_joy_bridge start_rc_web_sim.sh
```

| Launch 文件 | 说明 |
|-------------|------|
| `sealien_mavlink_joy_bridge.launch.py` | 仅 `io_joy_bridge` |
| `sealien_mavlink_joy_bridge_sim.launch.py` | `io_joy_bridge` + `rc_web_sim_server` |
| `sealien_mavlink_joy_bridge_obc_sim.launch.py` | 与 sim 相同（历史别名） |

浏览器访问：`http://<主机IP>:8080/` — 24 路 AI 滑条、24 路 DI 按钮；勾选自动发送后以约 50 Hz 向 `udp_target_host:udp_target_port` 发包。

自定义参数示例：

```bash
ros2 launch sealien_mavlink_joy_bridge sealien_mavlink_joy_bridge.launch.py \
  --ros-args -p listen_port:=14551 -p joy_topic:=/my_joy
```

或直接运行节点并指定参数文件：

```bash
ros2 run sealien_mavlink_joy_bridge io_joy_bridge --ros-args \
  --params-file $(ros2 pkg prefix sealien_mavlink_joy_bridge)/share/sealien_mavlink_joy_bridge/config/io_joy_bridge.yaml
```

## ROS 话题

| 话题（默认） | 类型 | 发布者 | 说明 |
|--------------|------|--------|------|
| `/joy` | `sensor_msgs/Joy` | `io_joy_bridge` | `axes` 长度 24，`buttons` 长度 24 |
| `/adc/raw` | `sealien_mavlink_joy_bridge/AdcRaw` | `mavlink_adc_bridge` | 仅遗留链路 |

`Joy` 映射规则（`io_joy_bridge`）：

- `axes[i]` = AI[i] 经死区处理后，范围钳位在 **[-1, 1]**
- `buttons[i]` = `(di >> i) & 1`，DI 为 `SEALIEN_IO_INPUT.di` 的 **低 24 位**
- `header.frame_id` = `"sealien_io"`

## 配置

### `config/io_joy_bridge.yaml`（主节点）

| 参数 | 默认 | 说明 |
|------|------|------|
| `listen_address` | `0.0.0.0` | UDP 绑定地址 |
| `listen_port` | `14550` | UDP 监听端口（与 MCU/模拟器目标一致） |
| `remote_ip_filter` | `""` | 非空时只接受该源 IP 的报文 |
| `joy_topic` | `/joy` | 发布话题名 |
| `publish_rate_hz` | `50.0` | 定时发布频率 |
| `link_timeout_ms` | `500` | 距上次有效 `SEALIEN_IO_INPUT` 超过此时间则**停止发布** |
| `deadzone` | `0.05` | AI 死区，\|v\| < deadzone 时输出 0 |

### `config/web_sim.yaml`（网页模拟）

| 参数 | 默认 | 说明 |
|------|------|------|
| `web_host` | `0.0.0.0` | HTTP 监听 |
| `web_port` | `8080` | HTTP 端口 |
| `udp_target_host` | `127.0.0.1` | MAVLink UDP 目标 |
| `udp_target_port` | `14550` | 须与 `io_joy_bridge.listen_port` 一致 |
| `send_rate_hz` | `50.0` | 自动发送频率 |

遗留配置：`config/mavlink_adc_bridge.yaml`、`config/adc_joy_mapper.yaml`、`config/default.yaml`（`mavlink_joy_bridge`）。

## 链路超时与失效行为

1. 在收到**至少一帧**有效 `SEALIEN_IO_INPUT` 之前，**不发布** `/joy`。
2. 收到数据后按 `publish_rate_hz` 持续发布最近一次 AI/DI。
3. 若超过 `link_timeout_ms` 未再收到报文，**停止发布**并打 `WARN` 日志；恢复接收后自动继续发布。

联调时请保证 MCU 或模拟源 **≥ 约 2 Hz** 持续发包（推荐 50 Hz），否则易触发超时。

## 验证

```bash
# 终端 1：桥接
ros2 launch sealien_mavlink_joy_bridge sealien_mavlink_joy_bridge.launch.py

# 终端 2：命令行发包（见下节）或 sim launch + 浏览器自动发送

ros2 topic echo /joy sensor_msgs/msg/Joy
ros2 topic hz /joy
```

## 命令行发包（`mavlink_io_udp_send`）

```bash
ros2 run sealien_mavlink_joy_bridge mavlink_io_udp_send \
  <host> <port> <di_mask> <ai0> <ai1> ... <ai23>
```

- `di_mask`：`uint32`，bit *i* = 1 表示 DI*i* 为高
- `ai*`：浮点 **[-1, 1]**，共 24 个

示例：DI1=1，AI3=0.5，其余为 0：

```bash
ros2 run sealien_mavlink_joy_bridge mavlink_io_udp_send \
  127.0.0.1 14550 2 \
  0 0 0 0.5 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
```

## MAVLink 协议

方言定义：`mavlink/sealien_rc.xml`（生成头文件在 `third_party/mavlink/sealien_rc/`）。

### `SEALIEN_IO_INPUT`（id **203**，当前使用）

| 字段 | 类型 | 说明 |
|------|------|------|
| seq | uint32 | 序号 |
| timestamp_ms | uint32 | 时间戳 (ms) |
| ai[24] | float[24] | 模拟量，归一化 **-1 .. 1** |
| di | uint32 | DI0..DI23 位图（实现仅使用低 24 位） |

### `SEALIEN_RC_INPUT`（id **202**，遗留）

16 通道 + mode + valid_mask；由 `mavlink_joy_bridge` / `mavlink_rc_udp_send` 使用，新硬件路径请用 **203**。

### MCU 侧建议

- 传输：UDP 单播至机载计算机 `listen_port`（默认 14550）
- 帧率：与下游一致，建议 **50 Hz**
- AI 发送前在 MCU 完成标定与 **[-1, 1]** 钳位；DI 打包为位图 `di`

## 自动化测试

```bash
~/sealien_ws/src/sealien_mavlink_joy_bridge/scripts/e2e_test.sh
```

脚本会启动 `io_joy_bridge`，用 `mavlink_io_udp_send` _burst_ 发送（AI3=0.5、DI1=1），并 `ros2 topic echo` 校验 `/joy` 一次。可通过环境变量 `SEALIEN_WS` 指定工作空间根目录。

## 目录结构

```text
sealien_mavlink_joy_bridge/
├── config/           # 节点参数
├── include/          # C++ 头文件
├── launch/           # launch 文件
├── mavlink/          # MAVLink XML 方言
├── msg/              # AdcRaw.msg（遗留）
├── scripts/          # e2e、网页启动脚本
├── src/              # 节点实现
├── tools/            # mavlink_*_udp_send 工具
├── web/              # 网页模拟静态资源
└── third_party/mavlink/
```

## 遗留节点说明

仍编译安装，便于旧固件或旧 onboardcontrol 配置迁移：

| 节点 | 配置 | 用途 |
|------|------|------|
| `mavlink_adc_bridge` | `mavlink_adc_bridge.yaml` | 解析多种 MAVLink RC/ADC 报文 → `/adc/raw` |
| `adc_joy_mapper` | `adc_joy_mapper.yaml` | PWM 标定、映射、可选 mode → `/joy` |
| `mavlink_joy_bridge` | `default.yaml` | `SEALIEN_RC_INPUT` + `channel_map` 一步到 `/joy` |

手动启动遗留链路示例：

```bash
ros2 run sealien_mavlink_joy_bridge mavlink_adc_bridge --ros-args \
  --params-file .../config/mavlink_adc_bridge.yaml
ros2 run sealien_mavlink_joy_bridge adc_joy_mapper --ros-args \
  --params-file .../config/adc_joy_mapper.yaml
```

## 许可证

Apache-2.0 — 维护者见 `package.xml`。
