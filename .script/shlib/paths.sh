#!/bin/bash
# ==============================================================================
# 脚本名称: paths.sh
# 描述: 统一路径管理配置文件。通过脚本位置自动推导工作空间根目录，
#       并定义 ROS、Colcon 及 Rerun 工具的各级目录变量。
# ==============================================================================

set -euo pipefail

# ------------------------------------------------------------------------------
# 1. 基础路径推导 (Workspace Root)
# ------------------------------------------------------------------------------

# 载入基础工具以使用 get_script_dir (原 script_dir)
# shellcheck source=common.sh
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"

# 定义脚本所在目录及工具链根目录
SHLIB_DIR="$(get_script_dir)"
SCRIPTS_DIR="$(realpath "$SHLIB_DIR/..")"

# 自动推导工作空间根目录 (WS_ROOT)
# 策略: 优先尝试 Git 根目录，若非 Git 环境则指向上级目录
if command -v git >/dev/null 2>&1 && git -C "$SCRIPTS_DIR" rev-parse --show-toplevel >/dev/null 2>&1; then
    DEFAULT_WS_ROOT="$(git -C "$SCRIPTS_DIR" rev-parse --show-toplevel)"
else
    DEFAULT_WS_ROOT="$(realpath "$SCRIPTS_DIR/..")"
fi

# 最终确定的工作空间根路径 (支持环境变量覆盖)
export WS_ROOT="${WS_ROOT:-$DEFAULT_WS_ROOT}"

# ------------------------------------------------------------------------------
# 2. ROS 与 环境配置路径
# ------------------------------------------------------------------------------

# ROS 底层安装路径 (如 humble, foxy 等)
export ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"

# 工作空间编译后的环境加载脚本
# 注意: 只有在编译成功后此文件才会存在
export WS_SETUP="${WS_SETUP:-$WS_ROOT/.colcon/install/setup.bash}"

# ------------------------------------------------------------------------------
# 3. Colcon 构建系统路径
# ------------------------------------------------------------------------------

# 将所有编译产物集中在隐藏的 .colcon 目录下，保持根目录整洁
export COLCON_ROOT="${COLCON_ROOT:-$WS_ROOT/.colcon}"
export COLCON_BUILD="${COLCON_BUILD:-$COLCON_ROOT/build}"
export COLCON_INSTALL="${COLCON_INSTALL:-$COLCON_ROOT/install}"
export COLCON_LOG="${COLCON_LOG:-$COLCON_ROOT/log}"

# ------------------------------------------------------------------------------
# 4. Rerun 运行时与日志路径
# ------------------------------------------------------------------------------

# Rerun 工具专用的持久化数据目录
export RERUN_HOME="${RERUN_HOME:-$WS_ROOT/.rerun}"
export RUN_BASE="${RUN_BASE:-$RERUN_HOME/runs}"
export LOG_BASE="${LOG_BASE:-$RERUN_HOME/logs}"

# 用于记录最近一次成功运行 ID 的索引文件
export LATEST_FILE="${LATEST_FILE:-$RUN_BASE/LATEST}"