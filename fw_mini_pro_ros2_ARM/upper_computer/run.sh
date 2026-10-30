#!/bin/bash
# 启动车辆上位机
# 用法:
#   ./run.sh              默认启动时软件使能
#   ./run.sh --no-enable  启动时不使能（手动按空格使能）

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

source /opt/ros/humble/setup.bash

# 兼容标准 ~/ros2_ws/src/can_client 布局和包直接位于工作空间根目录的布局。
for workspace_setup in \
    "$PACKAGE_DIR/../../install/setup.bash" \
    "$PACKAGE_DIR/../install/setup.bash"; do
    if [[ -f "$workspace_setup" ]]; then
        source "$workspace_setup"
        break
    fi
done

if ! ros2 pkg prefix can_client >/dev/null 2>&1; then
    echo "未找到 ROS2 包 can_client。请先使用 colcon build 编译并 source install/setup.bash。" >&2
    exit 1
fi

echo "======================================"
echo "         车辆上位机 操作说明"
echo "======================================"
echo "  ↑ / ↓       X轴速度 ±0.1m/s"
echo "  ← / →       转向角速度 / 横移速度 ±0.1"
echo "  空格         切换使能 / 禁用"
echo "  G            切换 4T4D / 横移档"
echo "  0            速度归零"
echo "  C            角速度归零"
echo "  R            安全停车解锁 / 故障复位"
echo "  S            紧急停车"
echo "  Q            退出"
echo "======================================"
echo

app_args=(--enable)
if [[ "${1:-}" == "--no-enable" ]]; then
    app_args=()
fi

exec ros2 run can_client app.py "${app_args[@]}"
