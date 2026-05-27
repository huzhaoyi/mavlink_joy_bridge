#!/bin/bash
# 单独启动 RC 网页模拟器（HTTP + UDP MAVLink，不依赖 rclpy）
set -e

WS="${SEALIEN_WS:-${HOME}/sealien_ws}"
PKG_SHARE="${WS}/install/sealien_mavlink_joy_bridge/share/sealien_mavlink_joy_bridge"
PKG_LIB="${WS}/install/sealien_mavlink_joy_bridge/lib/sealien_mavlink_joy_bridge"

if [[ ! -f "${WS}/install/setup.bash" ]]; then
    echo "错误: 未找到 ${WS}/install/setup.bash，请先 colcon build sealien_mavlink_joy_bridge" >&2
    exit 1
fi

# shellcheck source=/dev/null
source "${WS}/install/setup.bash"

CONFIG="${PKG_SHARE}/config/web_sim.yaml"
SCRIPT="${PKG_LIB}/rc_web_sim_server.py"

if [[ ! -f "${SCRIPT}" ]]; then
    echo "错误: 未找到 ${SCRIPT}" >&2
    exit 1
fi

WEB_HOST=""
WEB_PORT=""
UDP_HOST=""
UDP_PORT=""

usage() {
    echo "用法: $(basename "$0") [选项]"
    echo "  启动网页 RC 模拟器，默认 http://0.0.0.0:8080/ → UDP 127.0.0.1:14550"
    echo ""
    echo "选项:"
    echo "  --web-host HOST    网页监听地址 (默认见 web_sim.yaml)"
    echo "  --web-port PORT    网页端口"
    echo "  --udp-host HOST    MAVLink UDP 目标"
    echo "  --udp-port PORT    MAVLink UDP 端口"
    echo "  -h, --help         显示帮助"
    echo ""
    echo "环境变量:"
    echo "  SEALIEN_WS         工作空间路径 (默认: ~/sealien_ws)"
    echo ""
    echo "说明: 需另起 mavlink_mixed_io_receiver + adc_to_joy_mapper；全链路见 README 或 sim launch。"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --web-host)
            WEB_HOST="$2"
            shift 2
            ;;
        --web-port)
            WEB_PORT="$2"
            shift 2
            ;;
        --udp-host)
            UDP_HOST="$2"
            shift 2
            ;;
        --udp-port)
            UDP_PORT="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "未知参数: $1" >&2
            usage
            exit 1
            ;;
    esac
done

ARGS=(/usr/bin/python3 "${SCRIPT}" --config "${CONFIG}")
if [[ -n "${WEB_HOST}" ]]; then
    ARGS+=(--web-host "${WEB_HOST}")
fi
if [[ -n "${WEB_PORT}" ]]; then
    ARGS+=(--web-port "${WEB_PORT}")
fi
if [[ -n "${UDP_HOST}" ]]; then
    ARGS+=(--udp-host "${UDP_HOST}")
fi
if [[ -n "${UDP_PORT}" ]]; then
    ARGS+=(--udp-port "${UDP_PORT}")
fi

echo "启动 Mixed IO 网页模拟器..."
echo "  配置: ${CONFIG}"
WEB_PORT_DISPLAY="${WEB_PORT:-8080}"
echo "  浏览器: http://127.0.0.1:${WEB_PORT_DISPLAY}/  (或本机 IP)"
echo "  UDP 目标: 见 web_sim.yaml 或上述 --udp-* 参数"
echo "  按 Ctrl+C 停止"
echo ""

if command -v ss >/dev/null 2>&1; then
    if ss -tln 2>/dev/null | grep -q ":${WEB_PORT_DISPLAY} "; then
        echo "错误: 端口 ${WEB_PORT_DISPLAY} 已被占用。" >&2
        echo "  查看占用: ss -tlnp | grep :${WEB_PORT_DISPLAY}" >&2
        echo "  结束旧进程: pkill -f rc_web_sim_server.py" >&2
        echo "  或换端口: $(basename "$0") --web-port 8081" >&2
        exit 1
    fi
fi

exec "${ARGS[@]}"
