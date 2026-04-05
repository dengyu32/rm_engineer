#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

set +u
if [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
fi
if [[ -f "$WORKSPACE_DIR/.colcon/install/setup.bash" ]]; then
    source "$WORKSPACE_DIR/.colcon/install/setup.bash"
elif [[ -f "$WORKSPACE_DIR/install/setup.bash" ]]; then
    source "$WORKSPACE_DIR/install/setup.bash"
fi
set -u

command -v ros2 >/dev/null 2>&1 || {
    echo "未找到 ros2，请先 source ROS 2/工作区环境。" >&2
    exit 1
}

# 3. 获取外部输入的参数（例如：第一个参数为 target_value）
# 如果启动时没给参数，则默认值为 1
TARGET_VALUE="${1:-1}"

echo "Starting node with parameter: $TARGET_VALUE"
ros2 param set /fake_system_node fake_intent_id "$TARGET_VALUE"
