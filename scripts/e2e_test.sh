#!/bin/bash
set -e
WS="${SEALIEN_WS:-${HOME}/sealien_ws}"
source "${WS}/install/setup.bash"

killall mavlink_mixed_io_receiver 2>/dev/null || true
killall adc_to_joy_mapper 2>/dev/null || true
pkill -f rc_web_sim_server 2>/dev/null || true
sleep 1

RECV_CFG="${WS}/install/sealien_mavlink_joy_bridge/share/sealien_mavlink_joy_bridge/config/mavlink_mixed_io_receiver.yaml"
MAP_CFG="${WS}/install/sealien_mavlink_joy_bridge/share/sealien_mavlink_joy_bridge/config/adc_to_joy_mapper.yaml"
SEND="${WS}/install/sealien_mavlink_joy_bridge/lib/sealien_mavlink_joy_bridge/mavlink_mixed_io_udp_send"

ros2 run sealien_mavlink_joy_bridge mavlink_mixed_io_receiver --ros-args --params-file "${RECV_CFG}" &
RECV_PID=$!
ros2 run sealien_mavlink_joy_bridge adc_to_joy_mapper --ros-args --params-file "${MAP_CFG}" &
MAP_PID=$!
sleep 2

ADC_ARGS="5.0 0.0 2.5 2.5 2.5 2.5 2.5 2.5 2.5 2.5 2.5 2.5 2.5 2.5 2.5 2.5 2.5 2.5 2.5 2.5 2.5 2.5 2.5 2.5"

echo "=== burst MIXED_IO_DATA (ADC0=5V, ADC1=0V, GPIO0/1=1) ==="
echo "--- /mixed_io/raw ---"
timeout 6 ros2 topic echo /mixed_io/raw sealien_mavlink_joy_bridge/msg/MixedIoRaw --once > /tmp/raw_e2e.txt 2>&1 &
RPID=$!
sleep 0.3
for _ in $(seq 1 80); do
    "${SEND}" 127.0.0.1 14550 3 0 ${ADC_ARGS}
    sleep 0.05
    if ! kill -0 ${RPID} 2>/dev/null; then break; fi
done
wait ${RPID} 2>/dev/null || true
cat /tmp/raw_e2e.txt

echo "--- /joy ---"
timeout 6 ros2 topic echo /joy sensor_msgs/msg/Joy --once > /tmp/joy_e2e.txt 2>&1 &
EPID=$!
sleep 0.3
for _ in $(seq 1 80); do
    "${SEND}" 127.0.0.1 14550 3 0 ${ADC_ARGS}
    sleep 0.05
    if ! kill -0 ${EPID} 2>/dev/null; then break; fi
done
wait ${EPID} 2>/dev/null || true
cat /tmp/joy_e2e.txt

kill ${MAP_PID} ${RECV_PID} 2>/dev/null || true
