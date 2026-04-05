#!/bin/bash
# ==============================================================================
# 脚本名称: rerun.sh
# 描述: 自动化 ROS 2 工作空间的构建、清理与多终端启动脚本。
#       支持物理机器人 (realsystem) 与仿真环境 (fakesystem) 的快速切换。
# ==============================================================================

set -euo pipefail

# ------------------------------------------------------------------------------
# 1. 初始化共享库与环境变量
# ------------------------------------------------------------------------------
SCRIPT_FILE="$(realpath "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$SCRIPT_FILE")"
SHLIB_DIR="$SCRIPT_DIR/shlib"
if [[ ! -d "$SHLIB_DIR" && -d "$SCRIPT_DIR/.script/shlib" ]]; then
    SHLIB_DIR="$SCRIPT_DIR/.script/shlib"
fi

# 确保所有依赖脚本都能被正确 source
[[ -d "$SHLIB_DIR" ]] || {
    echo "[FATAL] 找不到 shlib 目录: $SHLIB_DIR" >&2
    exit 1
}

source "$SHLIB_DIR/common.sh"
source "$SHLIB_DIR/paths.sh"
source "$SHLIB_DIR/logging.sh"
source "$SHLIB_DIR/term_launch.sh"
source "$SHLIB_DIR/managed_processes.sh"

# ------------------------------------------------------------------------------
# 2. 帮助信息 (Usage)
# ------------------------------------------------------------------------------
usage() {
    cat <<'EOF'
用法:
  ./rerun.sh [SYSTEM_TYPE] [OPTIONS]

系统类型 (SYSTEM_TYPE):
  fakesystem (默认)   启动基础节点 + 模拟系统 (不占用硬件串口)
  realsystem          启动基础节点 + 硬件 USB_CDC 串口通讯

选项:
  --build-only        仅编译项目，完成后退出
  --run-only          跳过编译阶段，直接使用现有 install 产物运行
  --clean             编译前清理 build/install/log 目录
  --packages-only p1  仅编译指定包及其依赖 (逗号分隔)
  --pkg <name>        添加单个需要编译的包 (可多次使用)
  --kill-prior        启动前杀掉上一次的 rerun 会话 (默认调用 shlib/pidfile_kill.sh)
  --kill-only         仅杀掉旧会话并退出
  -h, --help          显示此帮助信息
EOF
}

# ------------------------------------------------------------------------------
# 3. 默认参数设置
# ------------------------------------------------------------------------------
SYSTEM="fakesystem"
BUILD_ONLY=0
RUN_ONLY=0
CLEAN=0
KILL_PRIOR=0
KILL_ONLY=0

PACKAGES_SELECT=()
PACKAGES_RESOLVED=()
FOXGLOVE_PORT="${FOXGLOVE_PORT:-8765}"
COLCON_BASE_PATHS=("${WS_ROOT}/src" "${WS_ROOT}/third_party")

# ------------------------------------------------------------------------------
# 4. 参数解析
# ------------------------------------------------------------------------------
if [[ "${1:-}" == "--" ]]; then shift; fi

while [[ $# -gt 0 ]]; do
    case "$1" in
        --fakesystem|fakesystem) SYSTEM="fakesystem"; shift ;;
        --realsystem|realsystem) SYSTEM="realsystem"; shift ;;
        --build-only)            BUILD_ONLY=1; shift ;;
        --run-only|--only-run)   RUN_ONLY=1; shift ;;
        --clean)                 CLEAN=1; shift ;;
        --packages-only|--package-only)
            shift
            [[ $# -eq 0 || "$1" == -* ]] && die "错误: --packages-only 需要包名列表"
            IFS=',' read -r -a _pkgs <<< "$1"
            for _pkg in "${_pkgs[@]}"; do [[ -n "$_pkg" ]] && PACKAGES_SELECT+=("$_pkg"); done
            shift ;;
        --package|--pkg)
            shift
            [[ $# -eq 0 || "$1" == -* ]] && die "错误: --package 需要包名"
            PACKAGES_SELECT+=("$1"); shift ;;
        --kill-prior)            KILL_PRIOR=1; shift ;;
        --kill-only)             KILL_PRIOR=1; KILL_ONLY=1; shift ;;
        -h|--help)               usage; exit 0 ;;
        *)                       print_color red "未知参数: $1"; usage; exit 1 ;;
    esac
done

# ------------------------------------------------------------------------------
# 5. 环境与逻辑校验
# ------------------------------------------------------------------------------
if [[ $RUN_ONLY -eq 1 ]]; then
    [[ $BUILD_ONLY -eq 1 ]] && die "--run-only 与 --build-only 不能同时使用"
    [[ $CLEAN -eq 1 ]] && die "--clean 模式下不能使用 --run-only"
fi

# ------------------------------------------------------------------------------
# 6. 会话清理 (允许 kill-only 在最小依赖下工作)
# ------------------------------------------------------------------------------
if [[ $KILL_PRIOR -eq 1 ]]; then
    if [[ -x "$SHLIB_DIR/pidfile_kill.sh" ]]; then
        # kill-only 的目标是“尽快释放旧会话占用”，不应依赖 ROS/Colcon/第三方组件状态。
        print_color yellow "正在清理旧会话记录..."
        "$SHLIB_DIR/pidfile_kill.sh"
    else
        warn "未发现 $SHLIB_DIR/pidfile_kill.sh，跳过自动清理。"
    fi
    [[ $KILL_ONLY -eq 1 ]] && exit 0
fi

# ------------------------------------------------------------------------------
# 7. 环境与依赖校验
# ------------------------------------------------------------------------------
# 只有真正要构建/运行新会话时，才进入较重的环境校验与依赖准备流程。
# gnome-terminal 不是硬依赖；无界面或缺失时 open_term 会退回后台模式。
require_cmd colcon ros2 python3 realpath
[[ ! -f "$ROS_SETUP" ]] && die "错误: 找不到 ROS 环境配置文件: $ROS_SETUP"

# 检查第三方依赖环境 (ONNX/RealSense)
source "$SHLIB_DIR/check_env.sh"
if [[ $RUN_ONLY -eq 1 ]]; then
    check_env verify || exit 1
else
    check_env prepare || exit 1
fi

# ------------------------------------------------------------------------------
# 8. 编译流程
# ------------------------------------------------------------------------------
cd "$WS_ROOT"

if [[ $RUN_ONLY -eq 0 ]]; then
    # 解析需要编译的包及其上游依赖
    if [[ ${#PACKAGES_SELECT[@]} -gt 0 ]]; then
        mapfile -t PACKAGES_RESOLVED < <(colcon list --names-only --packages-up-to "${PACKAGES_SELECT[@]}" --base-paths "${COLCON_BASE_PATHS[@]}")
    fi

    # 执行清理逻辑
    if [[ $CLEAN -eq 1 ]]; then
        print_color yellow "正在执行清理..."
        if [[ ${#PACKAGES_RESOLVED[@]} -gt 0 ]]; then
            for _pkg in "${PACKAGES_RESOLVED[@]}"; do
                rm -rf "$COLCON_BUILD/$_pkg" "$COLCON_INSTALL/$_pkg"
            done
        else
            rm -rf "$COLCON_BUILD" "$COLCON_INSTALL" "$COLCON_LOG"
        fi
    fi

    # 准备编译环境变量 (临时刷新)
    unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
    set +u; source "$ROS_SETUP"; set -u

    colcon_args=()
    [[ ${#PACKAGES_RESOLVED[@]} -gt 0 ]] && colcon_args+=(--packages-select "${PACKAGES_RESOLVED[@]}")

    print_color cyan "开始构建项目..."
    if colcon --log-base "$COLCON_LOG" build --symlink-install \
        --build-base "$COLCON_BUILD" \
        --install-base "$COLCON_INSTALL" \
        --base-paths "${COLCON_BASE_PATHS[@]}" \
        "${colcon_args[@]}" \
        --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --no-warn-unused-cli; then
        
        print_color green "构建成功！"
        # 自动合并编译数据库供 clangd 使用
        if [[ -x "$SHLIB_DIR/gen_compile_commands.sh" ]]; then
            "$SHLIB_DIR/gen_compile_commands.sh"
        fi
    else
        die "构建失败，请检查上方日志。"
    fi
fi

[[ $BUILD_ONLY -eq 1 ]] && exit 0

# ------------------------------------------------------------------------------
# 9. 运行准备 (生成唯一运行 ID)
# ------------------------------------------------------------------------------
mkdir -p "$RUN_BASE" "$LOG_BASE"
RUN_ID="$(date +%Y%m%d_%H%M%S_%N)"
RUN_DIR="$RUN_BASE/$RUN_ID"
mkdir -p "$RUN_DIR"
echo "$RUN_DIR" > "$LATEST_FILE"

# ------------------------------------------------------------------------------
# 10. 启动节点逻辑
# ------------------------------------------------------------------------------
launch_term() {
    # 参数: 1.窗口标题 2.执行命令
    open_term "$1" "$2" "$ROS_SETUP" "$WS_SETUP" "$RUN_DIR" "$LOG_BASE"
}

print_color cyan "=============================================="
print_color cyan " 运行模式: $SYSTEM"
print_color cyan " 运行 ID  : $RUN_ID"
print_color cyan "=============================================="

while IFS='|' read -r title cmd delay_after; do
    [[ -n "${title:-}" && -n "${cmd:-}" ]] || continue
    launch_term "$title" "$cmd"
    if [[ -n "${delay_after:-}" && "$delay_after" != "0" ]]; then
        sleep "$delay_after"
    fi
done < <(rerun_emit_launch_specs "$SYSTEM")

print_color green "所有节点已尝试启动。日志记录于: $RUN_DIR"
