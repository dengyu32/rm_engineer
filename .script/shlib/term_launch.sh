#!/bin/bash
# ==============================================================================
# 脚本名称: term_launch.sh
# 描述: 终端启动辅助工具。支持在独立 GNOME 终端中启动 ROS 节点，
#       并自动处理 PID 记录、日志双重重定向（文件+终端）以及 X11 授权。
# ==============================================================================

set -euo pipefail

# ------------------------------------------------------------------------------
# 1. 环境初始化
# ------------------------------------------------------------------------------
SHLIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=common.sh
# shellcheck source=logging.sh
# shellcheck source=paths.sh
source "$SHLIB_DIR/common.sh"
source "$SHLIB_DIR/logging.sh"
source "$SHLIB_DIR/paths.sh"

# ------------------------------------------------------------------------------
# 2. 核心函数: open_term
# ------------------------------------------------------------------------------
# 参数: $1 - 窗口标题, $2 - 执行命令, $3 - ROS 环境, $4 - WS 环境, $5 - 运行目录, $6 - 日志根目录
open_term() {
    local title="${1:?未指定标题}"
    local cmd="${2:?未指定命令}"
    local ros_setup="${3:?未指定 ROS setup}"
    local ws_setup="${4:?未指定 WS setup}"
    local run_dir="${5:?未指定运行目录}"
    local log_base="${6:?未指定日志目录}"

    require_cmd gnome-terminal tee mkfifo setsid ps

    # 清理标题以适配文件名
    local safe_title
    safe_title="$(sanitize_title "$title")"

    # --- 路径与文件定义 ---
    local launch_pid_file="$run_dir/launch_${safe_title}.pid"
    local launch_pgid_file="$run_dir/launch_${safe_title}.pgid"
    local launch_sid_file="$run_dir/launch_${safe_title}.sid"
    local term_pid_file="$run_dir/term_${safe_title}.pid"
    local fifo_file="$run_dir/pipe_${safe_title}.fifo"
    local runner_file="$run_dir/runner_${safe_title}.sh"
    local base_log_file="$log_base/${safe_title}.log"
    local run_log_file="$run_dir/${safe_title}.log"

    mkdir -p "$run_dir" "$log_base"

    # 初始化日志页眉
    log_header "$base_log_file" "$title" "$cmd"
    log_header "$run_log_file" "$title" "$cmd"

    # --------------------------------------------------------------------------
    # 3. 生成内部运行脚本 (Runner Script)
    # --------------------------------------------------------------------------
    # 该脚本将在新打开的终端中执行
    cat >"$runner_file" <<EOF
#!/bin/bash
set -euo pipefail
set -m  # 启用作业控制

source "$SHLIB_DIR/common.sh"
source "$SHLIB_DIR/logging.sh"

# 加载 ROS 环境 (临时关闭 set -u 以兼容原始 setup 脚本)
set +u
source "$ros_setup"
source "$ws_setup"
set -u

print_color cyan "[$title] 正在启动..."
echo \$\$ >"$term_pid_file"

# 管道处理：实现日志同时输出到终端和两个日志文件
fifo="$fifo_file"
[[ -p "\$fifo" ]] || mkfifo "\$fifo"
tee -a "$base_log_file" "$run_log_file" <"\$fifo" &
tee_pid=\$!

# 使用 setsid 启动目标命令，确保脱离终端控制
setsid bash -c '$cmd' >"\$fifo" 2>&1 &
launch_pid=\$!

# 记录多级 PID 以便后续精准清理 (KILL_PRIOR)
pgid=\$(ps --no-headers -o pgid= -p "\$launch_pid" 2>/dev/null | tr -d ' ' || true)
sid=\$(ps --no-headers -o sid= -p "\$launch_pid" 2>/dev/null | tr -d ' ' || true)
echo \$launch_pid >"$launch_pid_file"
echo \${pgid:-\$launch_pid} >"$launch_pgid_file"
echo \${sid:-\$launch_pid} >"$launch_sid_file"

# 等待主命令结束
set +e
wait "\$launch_pid"
exit_code=\$?
set -e

# 记录日志页脚
log_footer "$base_log_file" "\$exit_code"
log_footer "$run_log_file" "\$exit_code"

# 清理资源
exec </dev/null
wait "\$tee_pid" >/dev/null 2>&1 || true
rm -f "\$fifo"

print_color yellow "[$title] 进程已退出，退出码: \$exit_code"

# 根据配置决定是否保留终端窗口
if [[ "\${RERUN_KEEP_SHELL_ON_EXIT:-1}" == "1" ]]; then
    exec bash
fi
EOF

    chmod +x "$runner_file"

    # --------------------------------------------------------------------------
    # 4. 终端启动调度 (GUI vs Headless)
    # --------------------------------------------------------------------------
    
    # 检查是否为无界面模式或缺少显示器
    if [[ "${RERUN_NO_TERM:-0}" == "1" || -z "${DISPLAY:-}" ]]; then
        print_color yellow "[open_term] 无界面模式启动: $title"
        RERUN_KEEP_SHELL_ON_EXIT=0 bash "$runner_file" >/dev/null 2>&1 &
        return
    fi

    # X11 授权修复：解决快速弹出大量窗口时的权限问题
    if [[ "${RERUN_XHOST_FIX:-1}" == "1" ]] && command -v xhost >/dev/null 2>&1; then
        local _user="${USER:-$(whoami)}"
        xhost +SI:localuser:"$_user" >/dev/null 2>&1 || true
    fi

    # 启动间隔延迟：防止 XServer 瞬时压力过大
    local term_delay="${RERUN_TERM_DELAY:-0.2}"
    if [[ "$term_delay" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]] && awk "BEGIN { exit !($term_delay > 0) }"; then
        sleep "$term_delay"
    fi

    # 尝试启动 GNOME 终端
    if ! gnome-terminal --window --title="$title" -- bash -ic "RERUN_KEEP_SHELL_ON_EXIT=1 bash '$runner_file'"; then
        print_color red "[open_term] gnome-terminal 启动失败，退回到后台模式: $title"
        RERUN_KEEP_SHELL_ON_EXIT=0 bash "$runner_file" >/dev/null 2>&1 &
    fi
}
