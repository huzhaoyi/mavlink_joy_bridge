# Raw mixed IO sample from MCU (decoded from MAVLink MIXED_IO_DATA, msgid=20).
# Producer: mavlink_mixed_io_receiver
# Consumer: adc_to_joy_mapper / 任意需要原始电气量的下游

std_msgs/Header header

uint32 mcu_timestamp_ms      # MCU rt_tick (ms), copied from MIXED_IO_DATA.timestamp_ms

# 链路状态
# true  : 在 link_timeout_ms 内收到过有效报文
# false : failsafe 帧，下游应忽略 adc_v / gpio_input_mask 内容并按安全位处理
bool link_ok

# ADC 电压（伏特），MCU 已完成分压还原
# 通道展平规则: [dev1_ch0..7, dev2_ch0..7, dev3_ch0..7] = 24 路
float32[] adc_v

# GPIO 输入位图，bit i 对应 GPIO i (0..63)
# bit  0..7  = Dev1 P0, bit  8..15 = Dev1 P1, bit 16..23 = Dev2 P0, bit 24..31 = Dev2 P1
# bit 32..39 = Dev3 P0, bit 40..47 = Dev3 P1, bit 48..55 = Dev4 P0, bit 56..63 = Dev4 P1
uint64 gpio_input_mask
