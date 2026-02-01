#!/bin/bash
set -euo pipefail

# ----------------------------------------------------------------------------
# paths.sh : unified path definitions
# ----------------------------------------------------------------------------

# shellcheck source=common.sh
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"

SHLIB_DIR="$(script_dir)"
SCRIPTS_DIR="$(realpath "$SHLIB_DIR/..")"
# 脚本位于 workspace/.script，工作区根为其上一级（优先使用 git 根）
if command -v git >/dev/null 2>&1 && git -C "$SCRIPTS_DIR" rev-parse --show-toplevel >/dev/null 2>&1; then
  DEFAULT_WS_ROOT="$(git -C "$SCRIPTS_DIR" rev-parse --show-toplevel)"
else
  DEFAULT_WS_ROOT="$(realpath "$SCRIPTS_DIR/..")"
fi

WS_ROOT="${WS_ROOT:-$DEFAULT_WS_ROOT}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"

COLCON_ROOT="${COLCON_ROOT:-$WS_ROOT/.colcon}"
COLCON_BUILD="${COLCON_BUILD:-$COLCON_ROOT/build}"
COLCON_INSTALL="${COLCON_INSTALL:-$COLCON_ROOT/install}"
COLCON_LOG="${COLCON_LOG:-$COLCON_ROOT/log}"

WS_SETUP="${WS_SETUP:-$COLCON_INSTALL/setup.bash}"

RERUN_HOME="$WS_ROOT/.rerun"
RUN_BASE="$RERUN_HOME/runs"
LOG_BASE="$RERUN_HOME/logs"
LATEST_FILE="$RUN_BASE/LATEST"
