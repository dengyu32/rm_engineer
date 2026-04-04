#!/bin/bash
# ==============================================================================
# 脚本名称: pidfile_kill.sh 
# 描述: 清理 rerun 管理过的 ROS 会话与历史残留进程。
#       先遍历 .rerun/runs 下所有运行记录，再按命令特征兜底补杀。
#       遵循 INT -> TERM -> KILL 优雅退出序列。
# ==============================================================================

set -euo pipefail

# ------------------------------------------------------------------------------
# 1. 环境加载
# ------------------------------------------------------------------------------
SHLIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=common.sh
source "$SHLIB_DIR/common.sh"
# shellcheck source=paths.sh
source "$SHLIB_DIR/paths.sh"

require_cmd ps kill realpath

SELF_PID="$$"
SELF_PPID="${PPID:-}"
SELF_PGID="$(ps --no-headers -o pgid= -p "$SELF_PID" 2>/dev/null | tr -d ' ' || true)"
PARENT_PGID=""
if [[ -n "$SELF_PPID" ]]; then
    PARENT_PGID="$(ps --no-headers -o pgid= -p "$SELF_PPID" 2>/dev/null | tr -d ' ' || true)"
fi
PROTECTED_PGID_NOTICE_KEYS=""

# ------------------------------------------------------------------------------
# 2. 核心清理函数
# ------------------------------------------------------------------------------

read_numeric_ids_from_dir() {
    local dir="${1:?未指定目录}"
    local pattern="${2:?未指定文件模式}"
    local -n out_ref="$3"
    local f id

    out_ref=()
    [[ -d "$dir" ]] || return 0

    shopt -s nullglob
    for f in "$dir"/$pattern; do
        id="$(tr -d '[:space:]' < "$f")"
        [[ "$id" =~ ^[0-9]+$ ]] && out_ref+=("$id")
    done
    shopt -u nullglob

    if [[ ${#out_ref[@]} -gt 0 ]]; then
        mapfile -t out_ref < <(printf '%s\n' "${out_ref[@]}" | awk '!seen[$0]++')
    fi

    return 0
}

collect_run_dirs() {
    local -n out_ref="$1"
    local dir

    out_ref=()

    [[ -d "$RUN_BASE" ]] || return 0

    shopt -s nullglob
    for dir in "$RUN_BASE"/*; do
        [[ -d "$dir" ]] || continue
        out_ref+=("$dir")
    done
    shopt -u nullglob

    if [[ ${#out_ref[@]} -gt 0 ]]; then
        mapfile -t out_ref < <(printf '%s\n' "${out_ref[@]}" | awk '!seen[$0]++' | sort)
    fi

    return 0
}

collect_ids_from_run_dirs() {
    local pattern="${1:?未指定文件模式}"
    local -n out_ref="$2"
    shift 2
    local run_dirs=("$@")
    local run_dir
    local -a ids=()
    local -a current=()

    out_ref=()
    [[ ${#run_dirs[@]} -eq 0 ]] && return 0

    for run_dir in "${run_dirs[@]}"; do
        read_numeric_ids_from_dir "$run_dir" "$pattern" current
        [[ ${#current[@]} -eq 0 ]] || ids+=("${current[@]}")
    done

    if [[ ${#ids[@]} -gt 0 ]]; then
        mapfile -t out_ref < <(printf '%s\n' "${ids[@]}" | awk '!seen[$0]++')
    fi

    return 0
}

pgid_is_protected() {
    local pgid="${1:?未指定 PGID}"

    [[ -n "$SELF_PGID" && "$pgid" == "$SELF_PGID" ]] && return 0
    [[ -n "$PARENT_PGID" && "$pgid" == "$PARENT_PGID" ]] && return 0
    return 1
}

filter_protected_pgids() {
    local -n input_ref="$1"
    local -n output_ref="$2"
    local pgid
    local notice_key

    output_ref=()
    for pgid in "${input_ref[@]}"; do
        [[ -n "$pgid" ]] || continue
        if pgid_is_protected "$pgid"; then
            notice_key="|$pgid|"
            if [[ "$PROTECTED_PGID_NOTICE_KEYS" != *"$notice_key"* ]]; then
                print_color yellow "跳过当前清理会话所在进程组，避免误杀当前终端。"
                PROTECTED_PGID_NOTICE_KEYS+="$notice_key"
            fi
            continue
        fi
        output_ref+=("$pgid")
    done

    return 0
}

pgid_alive() {
    local pgid="${1:?未指定 PGID}"
    # 不用 ps -g 做存活判断，直接比对 PGID 列，避免不同 ps 实现下语义歧义。
    ps -eo pgid= | awk -v pgid="$pgid" '$1 == pgid { found=1; exit } END { exit !found }'
}

pid_alive() {
    local pid="${1:?未指定 PID}"
    kill -0 "$pid" 2>/dev/null
}

collect_session_pids() {
    local sid
    [[ $# -eq 0 ]] && return 0

    # 有些 ROS 节点会在收到组信号后继续派生/留存同一 session 内的子进程，后续再补清一次。
    for sid in "$@"; do
        ps -eo pid=,sid= | awk -v sid="$sid" '$2 == sid { print $1 }'
    done | awk -v self="$$" '$1 != self && !seen[$1]++'
}

# 按信号分阶段杀死进程组 (PGID)
kill_group_stage() {
    local sig="$1"
    local wait_s="$2"
    shift 2
    local pgids=("$@")
    local pgid

    [[ ${#pgids[@]} -eq 0 ]] && return

    for pgid in "${pgids[@]}"; do
        # 负号表示向整个进程组发送信号
        kill -s "$sig" -- "-$pgid" 2>/dev/null || true
    done

    # 等待进程退场
    local elapsed=0
    while [[ $elapsed -lt $wait_s ]]; do
        local any_alive=0
        for pgid in "${pgids[@]}"; do
            if pgid_alive "$pgid"; then any_alive=1; break; fi
        done
        [[ $any_alive -eq 0 ]] && break
        sleep 1
        elapsed=$((elapsed + 1))
    done

    return 0
}

kill_pid_stage() {
    local sig="$1"
    local wait_s="$2"
    shift 2
    local pids=("$@")
    local pid

    [[ ${#pids[@]} -eq 0 ]] && return

    for pid in "${pids[@]}"; do
        kill -s "$sig" "$pid" 2>/dev/null || true
    done

    local elapsed=0
    while [[ $elapsed -lt $wait_s ]]; do
        local any_alive=0
        for pid in "${pids[@]}"; do
            if pid_alive "$pid"; then any_alive=1; break; fi
        done
        [[ $any_alive -eq 0 ]] && break
        sleep 1
        elapsed=$((elapsed + 1))
    done

    return 0
}

collect_cmd_rule_pgids() {
    local pattern="${1:?未指定命令模式}"
    local -n out_ref="$2"

    out_ref=()
    local -a raw_pgids=()

    mapfile -t raw_pgids < <(
        ps -eo pid=,pgid=,cmd= \
            | awk -v pat="$pattern" -v self="$SELF_PID" '
                index($0, pat) > 0 && $1 != self {
                    gsub(/^[[:space:]]+|[[:space:]]+$/, "", $2)
                    if ($2 ~ /^[0-9]+$/) print $2
                }
            ' \
            | awk '!seen[$0]++'
    )

    filter_protected_pgids raw_pgids out_ref

    return 0
}

collect_port_rule_pids() {
    local keyword="${1:?未指定关键字}"
    local port="${2:?未指定端口}"
    local -n out_ref="$3"

    out_ref=()
    [[ "$port" =~ ^[0-9]+$ ]] || return 0

    if command -v ss >/dev/null 2>&1; then
        mapfile -t out_ref < <(
            ss -ltnp "sport = :$port" 2>/dev/null \
                | awk -F 'pid=' -v keyword="$keyword" 'index($0, keyword) > 0 { split($2, a, /[,)]/); if (a[1] ~ /^[0-9]+$/) print a[1] }' \
                | awk '!seen[$0]++'
        )
    elif command -v lsof >/dev/null 2>&1; then
        mapfile -t out_ref < <(
            lsof -tiTCP:"$port" -sTCP:LISTEN 2>/dev/null \
                | while read -r pid; do
                    ps -p "$pid" -o cmd= 2>/dev/null | grep -F -q "$keyword" && echo "$pid"
                done \
                | awk '!seen[$0]++'
        )
    fi

    return 0
}

kill_port_fallbacks() {
    local rules="${RERUN_PORT_CLEANUP_RULES:-foxglove_bridge:${FOXGLOVE_PORT:-8765}:foxglove_bridge}"
    local rule label port keyword
    local -a pids=()

    IFS=';' read -r -a rule_list <<< "$rules"
    for rule in "${rule_list[@]}"; do
        [[ -n "$rule" ]] || continue
        IFS=':' read -r label port keyword <<< "$rule"
        [[ -n "${label:-}" && -n "${port:-}" && -n "${keyword:-}" ]] || continue

        # 端口兜底保留为轻量规则表，默认只覆盖 foxglove_bridge，后续需要时再增条目即可。
        collect_port_rule_pids "$keyword" "$port" pids

        [[ ${#pids[@]} -eq 0 ]] && continue

        print_color yellow "检测到 ${label} 仍占用端口 $port，执行兜底清理: ${pids[*]}"
        kill_pid_stage TERM 2 "${pids[@]}"
        kill_pid_stage KILL 0 "${pids[@]}"
    done

    return 0
}

kill_process_fallbacks() {
    local rules="${RERUN_PROCESS_CLEANUP_RULES:-engineer_bringup:ros2 launch engineer_bringup base_bringup.launch.py;auto_node_launch:ros2 launch auto_node start_auto_node.launch.py;teleop_launch:ros2 launch teleop_node start_teleop_node.launch.py;vision_launch:ros2 launch detect_node detect.launch.py;foxglove_launch:ros2 launch foxglove_bridge foxglove_bridge_launch.xml;fake_system_launch:ros2 launch fake_system fake_system_node.launch.py;usb_cdc_launch:ros2 launch usb_cdc usb_cdc_node.launch.py;arm_solve_server:/arm_solve_server_node --ros-args;move_group:/move_group --ros-args;object_load:/object_load --ros-args;robot_state_publisher:/robot_state_publisher --ros-args;static_tf:/static_transform_publisher --ros-args;auto_node_exec:/auto_node_main --ros-args;teleop_exec:/teleop_node --ros-args;fake_system_exec:/fake_system_node --ros-args;usb_cdc_exec:/usb_cdc_node --ros-args;vision_container:/component_container_mt --ros-args}"
    local rule label pattern
    local -a pgids=()

    IFS=';' read -r -a rule_list <<< "$rules"
    for rule in "${rule_list[@]}"; do
        [[ -n "$rule" ]] || continue
        IFS=':' read -r label pattern <<< "$rule"
        [[ -n "${label:-}" && -n "${pattern:-}" ]] || continue

        collect_cmd_rule_pgids "$pattern" pgids
        [[ ${#pgids[@]} -eq 0 ]] && continue

        print_color yellow "检测到 ${label} 历史残留进程组，执行兜底清理: ${pgids[*]}"
        kill_group_stage INT 2 "${pgids[@]}"
        kill_group_stage TERM 2 "${pgids[@]}"
        kill_group_stage KILL 0 "${pgids[@]}"
    done

    return 0
}

cleanup_run_dir_if_gone() {
    local run_dir="${1:?未指定运行目录}"
    local -a pgids=()
    local -a sids=()
    local -a term_pids=()
    local -a session_pids=()
    local pgid pid
    local keep_dir=0

    [[ -d "$run_dir" ]] || return 0

    read_numeric_ids_from_dir "$run_dir" "launch_*.pgid" pgids
    for pgid in "${pgids[@]}"; do
        if pgid_alive "$pgid"; then
            keep_dir=1
            break
        fi
    done

    if [[ $keep_dir -eq 0 ]]; then
        read_numeric_ids_from_dir "$run_dir" "launch_*.sid" sids
        if [[ ${#sids[@]} -gt 0 ]]; then
            mapfile -t session_pids < <(collect_session_pids "${sids[@]}")
            [[ ${#session_pids[@]} -eq 0 ]] || keep_dir=1
        fi
    fi

    if [[ $keep_dir -eq 0 ]]; then
        read_numeric_ids_from_dir "$run_dir" "term_*.pid" term_pids
        for pid in "${term_pids[@]}"; do
            if pid_alive "$pid"; then
                keep_dir=1
                break
            fi
        done
    fi

    if [[ $keep_dir -eq 0 ]]; then
        print_color green "已清理对应运行目录: $run_dir"
        rm -rf "$run_dir"
    else
        print_color yellow "运行目录仍有关联进程存活，暂不删除: $run_dir"
    fi

    return 0
}

# ------------------------------------------------------------------------------
# 3. 执行流程
# ------------------------------------------------------------------------------

# 1. 扫描所有历史运行目录，而不是只清理 LATEST 指向的那一轮。
collect_run_dirs RUN_DIRS

if [[ ${#RUN_DIRS[@]} -gt 0 ]]; then
    for run_dir in "${RUN_DIRS[@]}"; do
        realpath_guard_prefix "$run_dir" "$RUN_BASE"
    done

    print_color cyan "发现历史运行目录 ${#RUN_DIRS[@]} 个，开始汇总清理。"
else
    print_color yellow "未发现历史运行目录，继续执行残留进程兜底扫描。"
fi

# 2. 汇总所有历史记录中的 PGID / SID / 终端 PID。
# PGID 用于主清理，SID 用于补抓残留会话进程，终端 PID 用于顺手关闭关联窗口。
collect_ids_from_run_dirs "launch_*.pgid" PGIDS "${RUN_DIRS[@]}"
collect_ids_from_run_dirs "launch_*.sid" SIDS "${RUN_DIRS[@]}"
collect_ids_from_run_dirs "term_*.pid" TERM_PIDS "${RUN_DIRS[@]}"
filter_protected_pgids PGIDS SAFE_PGIDS

# 3. 开始按记录清理
if [[ ${#SAFE_PGIDS[@]} -gt 0 ]]; then
    # 先给 ROS launch / 节点优雅退出机会，再逐级升级信号强度。
    print_color cyan "正在清理会话进程组: ${SAFE_PGIDS[*]} ..."
    kill_group_stage INT 3 "${SAFE_PGIDS[@]}"
    kill_group_stage TERM 2 "${SAFE_PGIDS[@]}"
    kill_group_stage KILL 0 "${SAFE_PGIDS[@]}"
fi

if [[ ${#SIDS[@]} -gt 0 ]]; then
    mapfile -t SESSION_PIDS < <(collect_session_pids "${SIDS[@]}")
    if [[ ${#SESSION_PIDS[@]} -gt 0 ]]; then
        # 处理“主组退了，但同 session 里还有残留子进程”的情况。
        print_color yellow "检测到会话残留进程，继续清理: ${SESSION_PIDS[*]}"
        kill_pid_stage TERM 2 "${SESSION_PIDS[@]}"
        kill_pid_stage KILL 0 "${SESSION_PIDS[@]}"
    fi
fi

if [[ ${#TERM_PIDS[@]} -gt 0 ]]; then
    print_color yellow "正在关闭关联终端..."
    for pid in "${TERM_PIDS[@]}"; do
        kill -TERM "$pid" 2>/dev/null || true
    done
fi

# 4. 后置清理
# 按运行目录逐个检查，仅删除其对应进程已清干净的目录。
if [[ ${#RUN_DIRS[@]} -gt 0 ]]; then
    for run_dir in "${RUN_DIRS[@]}"; do
        cleanup_run_dir_if_gone "$run_dir"
    done
fi

if [[ -f "$LATEST_FILE" ]]; then
    local_latest_dir=""
    local_latest_dir="$(cat "$LATEST_FILE" 2>/dev/null || true)"
    if [[ -z "$local_latest_dir" || ! -d "$local_latest_dir" ]]; then
        rm -f "$LATEST_FILE"
    fi
fi

# 刷新 ROS 2 状态
if command -v ros2 >/dev/null 2>&1; then
    ros2 daemon stop >/dev/null 2>&1 || true
fi

# 5. 最后一层保险：如果有没记进 pid 文件的旧进程，也按命令特征补杀。
kill_process_fallbacks

# 6. 如果规则表中的服务仍占端口，直接对监听进程定向收尾。
kill_port_fallbacks

print_color green "清理完成"
