#!/bin/bash
set -e
WS="${SEALIEN_WS:-${HOME}/sealien_ws}"
source "${WS}/install/setup.bash"

killall io_joy_bridge 2>/dev/null || true
pkill -f rc_web_sim_server 2>/dev/null || true
sleep 1

CFG="${WS}/install/sealien_mavlink_joy_bridge/share/sealien_mavlink_joy_bridge/config/io_joy_bridge.yaml"
SEND="${WS}/install/sealien_mavlink_joy_bridge/lib/sealien_mavlink_joy_bridge/mavlink_io_udp_send"

ros2 run sealien_mavlink_joy_bridge io_joy_bridge --ros-args --params-file "${CFG}" &
BRIDGE_PID=$!
sleep 2

DI_MASK=2
AI_ARGS="0 0 0 0.5 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0"

echo "=== burst SEALIEN_IO_INPUT (AI3=0.5, DI1=1) ==="
timeout 6 ros2 topic echo /joy sensor_msgs/msg/Joy --once > /tmp/joy_e2e.txt 2>&1 &
EPID=$!
sleep 0.3
while kill -0 ${EPID} 2>/dev/null; do
    "${SEND}" 127.0.0.1 14550 "${DI_MASK}" ${AI_ARGS}
    sleep 0.05
done
wait ${EPID} 2>/dev/null || true
cat /tmp/joy_e2e.txt

kill ${BRIDGE_PID} 2>/dev/null || true
