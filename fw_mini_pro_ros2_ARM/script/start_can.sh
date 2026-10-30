#!/usr/bin/env bash

# 手动配置并启动 SocketCAN 接口。
# 用法: ./script/start_can.sh [接口名] [波特率]

set -Eeuo pipefail

readonly CAN_IFACE="${1:-can0}"
readonly CAN_BITRATE="${2:-500000}"

die() {
    echo "[start_can] 错误: $*" >&2
    exit 1
}

if (( $# > 2 )); then
    die "参数过多。用法: $0 [接口名] [波特率]"
fi

command -v ip >/dev/null 2>&1 || die "未找到 ip 命令，请安装 iproute2。"
command -v sudo >/dev/null 2>&1 || die "未找到 sudo 命令。"

[[ "$CAN_IFACE" =~ ^[[:alnum:]_.-]+$ ]] || \
    die "接口名 '$CAN_IFACE' 非法。"
[[ "$CAN_BITRATE" =~ ^[1-9][0-9]*$ ]] || \
    die "波特率 '$CAN_BITRATE' 必须是正整数。"

ip link show dev "$CAN_IFACE" >/dev/null 2>&1 || \
    die "网络接口 '$CAN_IFACE' 不存在。"

echo "[start_can] 正在配置 $CAN_IFACE，波特率 $CAN_BITRATE bit/s..."
sudo -v || die "无法获取 sudo 权限。"
sudo ip link set dev "$CAN_IFACE" down || \
    die "无法关闭接口 '$CAN_IFACE'。"
sudo ip link set dev "$CAN_IFACE" type can bitrate "$CAN_BITRATE" || \
    die "无法将 '$CAN_IFACE' 的波特率设置为 $CAN_BITRATE。"
sudo ip link set dev "$CAN_IFACE" up || \
    die "无法启动接口 '$CAN_IFACE'。"

details="$(ip -details link show dev "$CAN_IFACE")" || \
    die "无法读取接口 '$CAN_IFACE' 的最终状态。"

readonly up_pattern='<[^>]*UP[^>]*>'
readonly stopped_pattern='can[[:space:]]+state[[:space:]]+(STOPPED|BUS-OFF)'
readonly bitrate_pattern="bitrate[[:space:]]+${CAN_BITRATE}([[:space:]]|$)"

[[ "$details" =~ $up_pattern ]] || \
    die "接口 '$CAN_IFACE' 配置后仍未处于 UP 状态。"
[[ ! "$details" =~ $stopped_pattern ]] || \
    die "接口 '$CAN_IFACE' 的 CAN 控制器仍为停止或 BUS-OFF 状态。"
[[ "$details" =~ $bitrate_pattern ]] || \
    die "接口 '$CAN_IFACE' 未报告目标波特率 $CAN_BITRATE。"

echo "[start_can] $CAN_IFACE 已成功启动。"
echo "$details"
