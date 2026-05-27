# sealien_mavlink_joy_bridge

ROS 2 包：接收机载 MCU 经 MAVLink **`MIXED_IO_DATA`**（msgid=20，与 sealien-ctrlcore 一致）上报的 **24 路 ADC 电压** 与 **GPIO 输入**，发布原始话题与 `sensor_msgs/Joy`。

## 三层架构

```text
MCU (ADS7128 ×3 + TCA9535 ×4)
    │  MIXED_IO_DATA, UDP (建议 ≥ 50 Hz)
    ▼
mavlink_mixed_io_receiver
    │  /mixed_io/raw  (sealien_mavlink_joy_bridge/msg/MixedIoRaw)
    │  事件驱动：每收到一帧发一次；超时持续发 link_ok=false
    ▼
adc_to_joy_mapper
    │  /joy           (sensor_msgs/Joy)
    │  per-channel 标定 + 双边死区 + expo + 断线检测 + invert + LPF
    ▼
下游控制
```

拆三层的目的：
- MAVLink/UDP 解析、joystick 校准两个关注点分离，换协议或换板不影响下游
- `/mixed_io/raw` 可单独录包/回放/单测
- 后续若需要 `joy_to_uinput` (→ `/dev/input/jsX`) 也只是再串一层

本机联调链路：

```text
网页 (web/) / mavlink_mixed_io_udp_send  ──UDP 14550──►  receiver  ─►  mapper  ─► /joy
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
# 接 MCU（receiver + mapper）
ros2 launch sealien_mavlink_joy_bridge sealien_mavlink_joy_bridge.launch.py

# receiver + mapper + 网页模拟（无需 MCU）
ros2 launch sealien_mavlink_joy_bridge sealien_mavlink_joy_bridge_sim.launch.py

# 仅网页（另开终端启动 receiver+mapper）
ros2 run sealien_mavlink_joy_bridge start_rc_web_sim.sh
```

浏览器：`http://<主机IP>:8080/` — 24 路 ADC (0~5 V)、24 路 GPIO 输入。

## 中间话题 `/mixed_io/raw`

`sealien_mavlink_joy_bridge/msg/MixedIoRaw`：

| 字段 | 含义 |
|------|------|
| `header.stamp` | 接收侧时间戳 |
| `mcu_timestamp_ms` | MCU `rt_tick`（ms） |
| `link_ok` | 链路是否活跃；`false` 表示 failsafe 帧 |
| `adc_v[24]` | ADC 电压 (V)；展平规则同 MCU |
| `gpio_input_mask` | uint64 位图，bit i 对应 GPIO i |

事件驱动：receiver 每解析一帧 `MIXED_IO_DATA` 立即发布一次；同时 50 Hz 定时检查超时，超时即持续发 `link_ok=false` 的全零帧。

## `/joy` 映射（mapper 内部）

mapper 订阅 `/mixed_io/raw`，按以下流水线生成 `axes`：

1. **三段线性标定**：以 `adc_center_v` 为中位，将 `[adc_min_v, adc_center_v]` 映射到 `[-1, 0]`，`[adc_center_v, adc_max_v]` 映射到 `[0, +1]`；超界自动 clamp
2. **双边死区**：`deadzone_low`/`deadzone_high` 作用在 `[-1, 1]` 输出域，死区外重新线性拉伸到端点
3. **Expo 曲线**：`out = sign(x) * (expo * |x|³ + (1-expo) * |x|)`，`expo ∈ [0,1]`，0=线性
4. **通道反向**：`invert[i]=true` 时取反
5. **一阶低通**：`filtered = lpf_alpha * new + (1-lpf_alpha) * prev`
6. **断线检测**：`adc_v` 超出 `[adc_valid_min_v, adc_valid_max_v]` 或 NaN/Inf → 该轴强制 0，并清 LPF 状态

`buttons` 来自 `gpio_input_mask`，按 `buttons_gpio_indices` / `buttons_joy_indices` / `buttons_active_high` 配置。

`link_ok=false` 帧：mapper 直接发 `axes=0 / buttons=0`，`frame_id="mixed_io_failsafe"`，并清零 LPF 状态。

## 配置

| 文件 | 说明 |
|------|------|
| `config/mavlink_mixed_io_receiver.yaml` | UDP 监听、`link_timeout_ms`、failsafe 频率 |
| `config/adc_to_joy_mapper.yaml` | 24 路标定、死区、expo、invert、LPF、断线阈值、通道映射 |
| `config/web_sim.yaml` | 网页 HTTP / UDP 目标 |

### 关键默认值与建议

| 项 | 默认 | 说明/建议 |
|----|------|-----------|
| `listen_port` | 14550 | MCU/网页 → receiver |
| `link_timeout_ms` | **200** | 手动控制范围 100~200 ms |
| `failsafe_publish_rate_hz` | 50 | 仅超时时持续发零；链路活时由 MCU 频率驱动 |
| MCU 上报频率 | — | 建议 **≥ 50 Hz**（MCU 当前默认 10 Hz 仅对慢控制够用） |
| `adc_min_v / center_v / max_v` | 0 / 2.5 / 5 V | 实测物理端点和中位，逐通道填 |
| `deadzone_low / high` | 0.05 | 输出域；中位漂移可以两边不对称 |
| `expo` | 0 | 线性 |
| `adc_valid_min_v / max_v` | -0.5 / 5.5 V | 用于断线检测 |
| `lpf_alpha` | 0.35 | 越大越快跟随，越小越平滑 |

### 实测电位器示例

杆只能到 0.42 V ~ 4.61 V，中位 2.47 V：

```yaml
adc_min_v:    [0.42, ...]
adc_center_v: [2.47, ...]
adc_max_v:    [4.61, ...]
```

这样推到底/顶 `axes` 仍是 ±1，中位漂移也被吸收。

## 验证

```bash
ros2 topic echo /mixed_io/raw
ros2 topic echo /joy
ros2 topic hz /joy
ros2 topic hz /mixed_io/raw     # 链路活时随 MCU；超时进入 50Hz failsafe

# 一键脚本：启动两节点 + UDP 发包 + 订阅 raw/joy + 验证 failsafe
bash ~/sealien_ws/install/sealien_mavlink_joy_bridge/lib/sealien_mavlink_joy_bridge/e2e_test.sh
```

发 24 路 ADC（ch0=0V, ch1=5V, 余 2.5V）+ GPIO bit0/1=1 时，链路活的期望片段：

```text
/mixed_io/raw: link_ok=True, frame=mixed_io, adc_v[:3]=[0.0, 5.0, 2.5], gpio_mask=0x3
/joy        : frame=mixed_io,         axes[:3]≈[-1, +1, 0],  buttons[:3]=[1, 1, 0]
```

停发后 200 ms 内自动切 failsafe：

```text
/mixed_io/raw: link_ok=False, frame=mixed_io_failsafe, adc_v 全 0
/joy        : frame=mixed_io_failsafe, axes 全 0, buttons 全 0
```

> LPF `lpf_alpha=0.35` 时，端点需连发约 15 帧才稳定到 ±1；想立刻到位把对应通道 `lpf_alpha` 调到 1.0 即可。

### 单测 mapper（绕过 receiver）

校准/曲线调参可以脱离 MCU/UDP，直接给中间话题灌数据：

```bash
ros2 run sealien_mavlink_joy_bridge adc_to_joy_mapper \
  --ros-args --params-file <pkg>/share/sealien_mavlink_joy_bridge/config/adc_to_joy_mapper.yaml

# 另一个终端持续灌 24 路 ADC（ch0=4.6V）
ros2 topic pub -r 50 /mixed_io/raw sealien_mavlink_joy_bridge/msg/MixedIoRaw \
  "{link_ok: true, mcu_timestamp_ms: 0,
    adc_v: [4.6, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5,
            2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5,
            2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5],
    gpio_input_mask: 0}"
```

## 命令行发包

```bash
ros2 run sealien_mavlink_joy_bridge mavlink_mixed_io_udp_send \
  <host> <port> <gpio_lo> <gpio_hi> <adc0_V> ... <adc23_V>
```

例：

```bash
ros2 run sealien_mavlink_joy_bridge mavlink_mixed_io_udp_send \
  127.0.0.1 14550 3 0 \
  0.0 5.0 2.5 2.5 2.5 2.5 2.5 2.5 \
  2.5 2.5 2.5 2.5 2.5 2.5 2.5 2.5 \
  2.5 2.5 2.5 2.5 2.5 2.5 2.5 2.5
```

## 故障排查

| 现象 | 处理 |
|------|------|
| `Address already in use` 8080 | `pkill -f rc_web_sim_server.py` 或 `start_rc_web_sim.sh --web-port 8081` |
| `Address already in use` 14550 | 旧 receiver 残留：`pkill -f mavlink_mixed_io_receiver` |
| `ros2 topic echo` 抛 `!rclpy.ok()` | ROS daemon 状态错乱：`ros2 daemon stop && ros2 daemon start` |
| `Package 'sealien_mavlink_joy_bridge' not found` | 当前 shell 未 `source install/setup.bash` |
| 看到旧 `AdcRaw` 接口 | 上次构建残留：`rm -rf build/sealien_mavlink_joy_bridge install/sealien_mavlink_joy_bridge` 后重建 |
| `/joy` 一直为 0 | 没收到 `MIXED_IO_DATA`，receiver 日志会有 `timeout (...) publishing failsafe`；检查 UDP 目标端口/防火墙 |
| `/joy` 端点不到 ±1 | LPF 收敛中（`lpf_alpha` 偏小）或电位器实际行程小于 `adc_min_v/adc_max_v` 配置；按【实测电位器示例】重标定 |
| 某通道始终为 0 | 触发了断线检测——检查电压是否落在 `[adc_valid_min_v, adc_valid_max_v]` 内 |

## MAVLink

- 方言头文件：`third_party/mavlink/sealien_mavlink/`（从 sealien-ctrlcore 同步）
- 本包仅使用 **`MIXED_IO_DATA`**（id 20）；下行 **`MIXED_IO_CMD`**（id 19）MCU 侧尚未实现接收

## 目录结构

```text
sealien_mavlink_joy_bridge/
├── msg/MixedIoRaw.msg
├── config/
│   ├── mavlink_mixed_io_receiver.yaml
│   ├── adc_to_joy_mapper.yaml
│   └── web_sim.yaml
├── include/sealien_mavlink_joy_bridge/
│   ├── mavlink_mixed_io_receiver.hpp
│   └── adc_to_joy_mapper.hpp
├── src/
│   ├── mavlink_mixed_io_receiver{,_node}.cpp
│   └── adc_to_joy_mapper{,_node}.cpp
├── tools/mavlink_mixed_io_udp_send.cpp
├── scripts/                      # rc_web_sim_server.py / start_rc_web_sim.sh / e2e_test.sh
├── launch/
├── web/
└── third_party/mavlink/sealien_mavlink/
```

## 许可证

Apache-2.0 — 维护者见 `package.xml`。
