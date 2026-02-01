#!/bin/bash
set -euo pipefail

# ----------------------------------------------------------------------------
# Bootstrap shared libs
# ----------------------------------------------------------------------------
SCRIPT_FILE="$(realpath "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$SCRIPT_FILE")"
SHLIB_DIR="$SCRIPT_DIR/shlib"
source "$SHLIB_DIR/common.sh"
source "$SHLIB_DIR/paths.sh"
source "$SHLIB_DIR/logging.sh"
source "$SHLIB_DIR/term_launch.sh"

usage() {
  cat <<'EOF'
Usage:
  ./rerun.sh [default fakesystem]
  ./rerun.sh --fakesystem
  ./rerun.sh --realsystem
  ./rerun.sh --build-only
  ./rerun.sh --packages-only pkg1,pkg2
  ./rerun.sh --package pkg1 --package pkg2

Options:
  --fakesystem, fakesystem        Launch: bringup + fake_system (no usb_cdc)
  --realsystem, realsystem        Launch: bringup + usb_cdc
  --build-only                    Only Clean + build then exit
  --packages-only pkg1,pkg2       Only build selected packages (comma-separated)
  --package, --pkg <name>         Add one package to the build-only list (repeatable)
  --kill-prior                    Kill previous rerun session (uses LATEST)
  --kill-only                     Kill previous rerun session (incl. rviz2) then exit
  -h, --help                      Show help

Default:
  If no system type is given, defaults to fakesystem.
EOF
}

# ----------------------------------------------------------------------------
# 默认参数
# ----------------------------------------------------------------------------
SYSTEM="fakesystem"
BUILD_ONLY=0
KILL_PRIOR=0
KILL_ONLY=0
# 仅编译指定包（colcon --packages-select）
PACKAGES_SELECT=()
# foxglove ws port (override via env FOXGLOVE_PORT)
FOXGLOVE_PORT="${FOXGLOVE_PORT:-8765}"

# ----------------------------------------------------------------------------
# 解析参数（兼容：-- fakesystem 这种写法）
# ----------------------------------------------------------------------------
if [[ "${1:-}" == "--" ]]; then
  shift || true
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --fakesystem|fakesystem)
      SYSTEM="fakesystem"
      shift
      ;;
    --realsystem|realsystem)
      SYSTEM="realsystem"
      shift
      ;;
    --build-only)
      BUILD_ONLY=1
      shift
      ;;
    --packages-only|--package-only)
      shift
      if [[ $# -eq 0 || "$1" == -* ]]; then
        die "--packages-only requires a comma-separated list, e.g. --packages-only pkg1,pkg2"
      fi
      IFS=',' read -r -a _pkgs <<< "$1"
      for _pkg in "${_pkgs[@]}"; do
        if [[ -n "$_pkg" ]]; then
          PACKAGES_SELECT+=("$_pkg")
        fi
      done
      shift
      ;;
    --package|--pkg)
      shift
      if [[ $# -eq 0 || "$1" == -* ]]; then
        die "--package requires a package name"
      fi
      PACKAGES_SELECT+=("$1")
      shift
      ;;
    --kill-prior)
      KILL_PRIOR=1
      shift
      ;;
    --kill-only)
      KILL_PRIOR=1
      KILL_ONLY=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      print_color red "Unknown option: $1"
      usage
      exit 1
      ;;
  esac
done

require_cmd gnome-terminal colcon ros2 python3 realpath
if [[ ! -f "$ROS_SETUP" ]]; then
  die "ROS setup not found: $ROS_SETUP"
fi

# ----------------------------------------------------------------------------
# rerun 会话清理：可选杀掉上一次会话
# ----------------------------------------------------------------------------
if [[ $KILL_PRIOR -eq 1 ]]; then
  kill_args=(--latest)
  if [[ $KILL_ONLY -eq 1 ]]; then
    kill_args+=(--kill-rviz)
  fi
  "$SCRIPTS_DIR/pidfile_kill.sh" "${kill_args[@]}"
  if [[ $KILL_ONLY -eq 1 ]]; then
    exit 0
  fi
fi

# ----------------------------------------------------------------------------
# 默认：清理 + 构建
# ----------------------------------------------------------------------------
cd "$WS_ROOT"

print_color green "Cleaning previous builds, install, logs ..."
rm -rf "$COLCON_BUILD" "$COLCON_INSTALL" "$COLCON_LOG" compile_commands.json

# ----------------------------------------------------------------------------
# 清理遗留的 prefix 环境，避免不存在的 install 路径导致 colcon 提示警告
# ----------------------------------------------------------------------------
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH

# ----------------------------------------------------------------------------
# 重新加载 ROS 环境，确保 rosidl / typesupport 等基础包可见
# ----------------------------------------------------------------------------
# 某些发行版 setup.* 会引用未定义变量；临时关闭 set -u 再恢复
set +u
source "$ROS_SETUP"
set -u

print_color green "Building workspace ..."
colcon_args=()
if [[ ${#PACKAGES_SELECT[@]} -gt 0 ]]; then
  # Validate package names early to avoid confusing colcon output
  mapfile -t _all_pkgs < <(colcon list --names-only)
  unknown_pkgs=()
  for _p in "${PACKAGES_SELECT[@]}"; do
    found=0
    for _ap in "${_all_pkgs[@]}"; do
      if [[ "$_p" == "$_ap" ]]; then
        found=1
        break
      fi
    done
    if [[ $found -eq 0 ]]; then
      unknown_pkgs+=("$_p")
    fi
  done
  if [[ ${#unknown_pkgs[@]} -gt 0 ]]; then
    print_color red "Unknown package(s): ${unknown_pkgs[*]}"
    print_color yellow "Available packages (from colcon list):"
    printf '  %s\n' "${_all_pkgs[@]}"
    exit 1
  fi
  print_color yellow "Packages-only: ${PACKAGES_SELECT[*]}"
  colcon_args+=(--packages-select "${PACKAGES_SELECT[@]}")
fi
colcon --log-base "$COLCON_LOG" build --symlink-install \
  --build-base "$COLCON_BUILD" \
  --install-base "$COLCON_INSTALL" \
  "${colcon_args[@]}" \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --no-warn-unused-cli

# ----------------------------------------------------------------------------
# 生成/合并 compile_commands.json 供 clangd 使用
# ----------------------------------------------------------------------------
if [[ -x "$SCRIPTS_DIR/gen_compile_commands.sh" ]]; then
  "$SCRIPTS_DIR/gen_compile_commands.sh"
else
  print_color yellow "gen_compile_commands.sh not found; skip clangd database merge"
fi

if [[ $BUILD_ONLY -eq 1 ]]; then
  print_color green "Build done (--build-only). Exit."
  exit 0
fi

# ----------------------------------------------------------------------------
# rerun 会话标识：用于 pidfile_kill.sh 精准清理（避免误杀）
# ----------------------------------------------------------------------------
mkdir -p "$RUN_BASE"
mkdir -p "$LOG_BASE"

RUN_ID="$(date +%Y%m%d_%H%M%S_%N)"
RUN_DIR="$RUN_BASE/$RUN_ID"
mkdir -p "$RUN_DIR"
echo "$RUN_DIR" > "$LATEST_FILE"

print_color yellow "rerun RUN_ID : $RUN_ID"
print_color yellow "rerun RUN_DIR: $RUN_DIR"
kill_cmd="$SCRIPTS_DIR/pidfile_kill.sh --latest"
print_color yellow "kill command : $kill_cmd"

# ----------------------------------------------------------------------------
# 运行前：打印最终配置
# ----------------------------------------------------------------------------
print_color cyan "=============================================="
print_color cyan "rerun config:"
print_color cyan "  workspace : $WS_ROOT"
print_color cyan "  system    : $SYSTEM"
print_color cyan "  foxglove  : port=$FOXGLOVE_PORT"
if [[ "$SYSTEM" == "realsystem" ]]; then
  print_color cyan "  bringup   : enabled"
  print_color cyan "  usb_cdc   : enabled"
  print_color cyan "  fake_node : disabled"
else
  print_color cyan "  bringup   : enabled"
  print_color cyan "  usb_cdc   : disabled"
  print_color cyan "  fake_node : enabled"
fi
print_color cyan "=============================================="

# ----------------------------------------------------------------------------
# 启动 bringup（两种模式都要）
# ----------------------------------------------------------------------------
print_color green "Start bringup.launch.py ..."
# open_term "engineer bringup" "ros2 launch engineer_bringup robot_bringup.launch.py"
open_term "engineer bringup" "ros2 launch engineer_bringup base_bringup.launch.py" "$ROS_SETUP" "$WS_SETUP" "$RUN_DIR" "$LOG_BASE" # 包括 arm_solve
open_term "servo container" "ros2 launch arm_servo servo_container.launch.py" "$ROS_SETUP" "$WS_SETUP" "$RUN_DIR" "$LOG_BASE"
open_term "top hfsm" "ros2 launch top_hfsm top_hfsm_node.launch.py" "$ROS_SETUP" "$WS_SETUP" "$RUN_DIR" "$LOG_BASE"
open_term "foxglove bridge" "ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=${FOXGLOVE_PORT}" "$ROS_SETUP" "$WS_SETUP" "$RUN_DIR" "$LOG_BASE"

# ----------------------------------------------------------------------------
# 模式差异
# ----------------------------------------------------------------------------
if [[ "$SYSTEM" == "fakesystem" ]]; then
  # fake：额外启动 fake_system_node
  print_color green "Start fake_system_node ... (fakesystem only)"
  open_term "fake system" "ros2 launch fake_system fake_system_node.launch.py" "$ROS_SETUP" "$WS_SETUP" "$RUN_DIR" "$LOG_BASE"
else
  # real：额外启动 usb_cdc
  print_color green "Open usb cdc node ... (realsystem only)"
  open_term "usb cdc" "ros2 launch usb_cdc usb_cdc_node.launch.py" "$ROS_SETUP" "$WS_SETUP" "$RUN_DIR" "$LOG_BASE"
fi
