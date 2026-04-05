#!/bin/bash
# ==============================================================================
# 脚本名称: managed_processes.sh
# 描述: 维护 rerun 启动项的唯一输入源，并从同一份定义派生清理规则。
# ==============================================================================

set -euo pipefail

# ------------------------------------------------------------------------------
# 1. 启动项定义
# ------------------------------------------------------------------------------
# 输出格式:
#   title|command|delay_after_sec
#
# 说明:
# - title: 窗口标题 / 日志标题
# - command: 实际执行命令
# - delay_after_sec: 启动后额外等待时间，可为空或 0

rerun_emit_launch_specs() {
    local system="${1:?未指定系统类型}"

    cat <<EOF
engineer bringup|ros2 launch engineer_bringup base_bringup.launch.py|1.0
auto node|ros2 launch auto_node start_auto_node.launch.py|0
teleop node|ros2 launch teleop_node start_teleop_node.launch.py|0
vision|ros2 launch detect_node vision.launch.py|0
foxglove bridge|ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=${FOXGLOVE_PORT:-8765}|0
EOF

    case "$system" in
        fakesystem)
            echo "fake system|ros2 launch fake_system fake_system_node.launch.py|0"
            ;;
        realsystem)
            echo "usb cdc|ros2 launch usb_cdc usb_cdc_node.launch.py|0"
            ;;
        *)
            echo "[FATAL] 未知系统类型: $system" >&2
            return 1
            ;;
    esac
}

rerun_emit_cleanup_launch_specs() {
    rerun_emit_launch_specs fakesystem
    rerun_emit_launch_specs realsystem
}

# ------------------------------------------------------------------------------
# 2. 派生清理规则
# ------------------------------------------------------------------------------
# 输出格式:
#   label:pattern

rerun_emit_cleanup_rules() {
    local line title cmd delay label

    while IFS='|' read -r title cmd delay; do
        [[ -n "${title:-}" && -n "${cmd:-}" ]] || continue
        label="$(echo "$title" | tr ' ' '_' )_launch"
        printf '%s:%s\n' "$label" "$cmd"
    done < <(rerun_emit_cleanup_launch_specs | awk '!seen[$0]++')

    cat <<'EOF'
arm_solve_server:/arm_solve_server_node --ros-args
move_group:/move_group --ros-args
object_load:/object_load --ros-args
robot_state_publisher:/robot_state_publisher --ros-args
static_tf:/static_transform_publisher --ros-args
auto_node_exec:/auto_node_main --ros-args
teleop_exec:/teleop_node --ros-args
fake_system_exec:/fake_system_node --ros-args
usb_cdc_exec:/usb_cdc_node --ros-args
vision_container:/component_container_mt --ros-args
EOF
}
