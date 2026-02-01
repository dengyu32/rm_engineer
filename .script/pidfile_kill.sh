#!/bin/bash
set -euo pipefail

# ============================================================================
# pidfile_kill.sh
#   按 rerun.sh 生成的 pid/pgid 文件精确清理上一次（或指定目录）的调试会话。
#   不使用关键词匹配，避免误杀其它 ROS2 会话。
# ----------------------------------------------------------------------------
# 用法：
#   ./pidfile_kill.sh --latest [--no-close-term] [--keep] [--dry-run]
#   ./pidfile_kill.sh --dir <RUN_DIR> [--no-close-term] [--keep] [--dry-run]
# ============================================================================

SCRIPT_FILE="$(realpath "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$SCRIPT_FILE")"
SHLIB_DIR="$SCRIPT_DIR/shlib"
source "$SHLIB_DIR/common.sh"
source "$SHLIB_DIR/paths.sh"

usage() {
  cat <<'EOF'
Usage:
  ./pidfile_kill.sh --latest [--no-close-term] [--keep] [--dry-run]
  ./pidfile_kill.sh --dir <RUN_DIR> [--no-close-term] [--keep] [--dry-run]
  # 额外：--kill-rviz 会额外清理当前用户的所有 rviz2 进程（匹配命令行）

Options:
  --latest       Kill the RUN_DIR recorded in .rerun/runs/LATEST (default)
  --dir DIR      Kill the specified RUN_DIR
  --no-close-term  Do NOT kill terminal bash shells (default: close them)
  --keep         Keep RUN_DIR after kill (default is to delete on success)
  --dry-run      Show targets only, do not send signals
  --kill-rviz    Also send INT/TERM/KILL to current user's rviz2 processes
  -h, --help     Show this help
EOF
}

# ----------------------------------------------------------------------------
# 参数解析
# ----------------------------------------------------------------------------
TARGET_DIR=""
USE_LATEST=1
CLOSE_TERM=1
KEEP_DIR=0
DRY_RUN=0
KILL_RVIZ=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --latest)
      USE_LATEST=1
      TARGET_DIR=""
      shift
      ;;
    --kill-rviz)
      KILL_RVIZ=1
      shift
      ;;
    --dir)
      USE_LATEST=0
      TARGET_DIR="${2:-}"
      if [[ -z "$TARGET_DIR" ]]; then
        print_color red "--dir requires a path"
        exit 1
      fi
      shift 2
      ;;
    --close-term) # kept for compatibility; default already closes
      CLOSE_TERM=1
      shift
      ;;
    --no-close-term)
      CLOSE_TERM=0
      shift
      ;;
    --keep)
      KEEP_DIR=1
      shift
      ;;
    --dry-run)
      DRY_RUN=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "Unknown option: $1"
      usage
      ;;
  esac
done

require_cmd ps kill realpath pkill pgrep

# 额外兜底：无论 pidfiles 是否存在，都尝试按常见节点名做模式匹配
EXTRA_PATTERNS_ALWAYS=(
  move_group
  moveit_simple_controller_manager
  arm_solve_server
  arm_servo
  top_hfsm
  fake_system_node
)

# ----------------------------------------------------------------------------
# 路径解析4
# ----------------------------------------------------------------------------
if [[ $USE_LATEST -eq 1 ]]; then
  if [[ -f "$LATEST_FILE" ]]; then
    TARGET_DIR="$(cat "$LATEST_FILE")"
  else
    warn "LATEST not found: $LATEST_FILE (will try fallback scan)"
    TARGET_DIR=""
  fi
fi

if [[ -n "$TARGET_DIR" && ! -d "$TARGET_DIR" ]]; then
  warn "No RUN_DIR to kill (stale LATEST): $TARGET_DIR"
  if [[ $USE_LATEST -eq 1 ]]; then
    rm -f "$LATEST_FILE"
  fi
  TARGET_DIR=""
fi

mkdir -p "$RUN_BASE"
if [[ -n "$TARGET_DIR" ]]; then
  print_color cyan "RUN_DIR: $TARGET_DIR"
  realpath_guard_prefix "$TARGET_DIR" "$RUN_BASE"
fi

# ----------------------------------------------------------------------------
# 工具函数
# ----------------------------------------------------------------------------
group_alive() {
  local pgid="$1"
  ps -o pid= --no-headers -g "$pgid" >/dev/null 2>&1 || return 1
  return 0
}

pid_alive() {
  local pid="$1"
  kill -0 "$pid" >/dev/null 2>&1
}

list_group() {
  local pgid="$1"
  ps -o pid=,pgid=,cmd= --no-headers -g "$pgid" 2>/dev/null | sed 's/^/  /' || true
}

session_alive() {
  local sid="$1"
  ps -o pid= --no-headers --sid "$sid" >/dev/null 2>&1
}

pattern_alive() {
  local pattern="$1"
  pgrep -u "$USER" -f "$pattern" >/dev/null 2>&1
}

pid_cwd_in_ws() {
  # Check whether the process current working directory is inside this workspace.
  local pid="$1"
  local cwd
  cwd="$(readlink -f "/proc/$pid/cwd" 2>/dev/null || true)"
  # 部分系统挂载了 hidepid，读取失败时无法准确判断；此时返回 true 避免漏杀。
  [[ -z "$cwd" || "$cwd" == "$WS_ROOT"* ]]
}

kill_sid_stage() {
  local sig="$1"
  local wait_s="$2"
  shift 2
  local sids=("$@")

  local sid
  for sid in "${sids[@]}"; do
    [[ -z "$sid" ]] && continue
    local pids=()
    while IFS= read -r pid; do
      [[ -z "$pid" ]] && continue
      pids+=("$pid")
    done < <(ps -o pid= --no-headers --sid "$sid" 2>/dev/null || true)
    for pid in "${pids[@]}"; do
      kill -s "$sig" "$pid" >/dev/null 2>&1 || true
    done
  done

  local elapsed=0
  while [[ $elapsed -lt $wait_s ]]; do
    local any=0
    for sid in "${sids[@]}"; do
      if ps -o pid= --no-headers --sid "$sid" 2>/dev/null | grep -q '[0-9]'; then
        any=1
        break
      fi
    done
    [[ $any -eq 0 ]] && break
    sleep 1
    elapsed=$((elapsed + 1))
  done
}

kill_pattern_stage() {
  local sig="$1"
  local wait_s="$2"
  local pattern="$3"

  pkill -u "$USER" "-${sig}" -f "$pattern" >/dev/null 2>&1 || true

  local elapsed=0
  while [[ $elapsed -lt $wait_s ]]; do
    if ! pattern_alive "$pattern"; then
      break
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done
}

kill_group_stage() {
  local sig="$1"
  local wait_s="$2"
  shift 2
  local pgids=("$@")

  local pgid
  for pgid in "${pgids[@]}"; do
    [[ -z "$pgid" ]] && continue
    # 负号发送给整个进程组；即便 ps 查不到也尝试发送
    kill -s "$sig" -- "-$pgid" >/dev/null 2>&1 || true
  done

  local elapsed=0
  while [[ $elapsed -lt $wait_s ]]; do
    local any=0
    for pgid in "${pgids[@]}"; do
      if group_alive "$pgid"; then
        any=1
        break
      fi
    done
    [[ $any -eq 0 ]] && break
    sleep 1
    elapsed=$((elapsed + 1))
  done
}

kill_pid_stage() {
  local sig="$1"
  local wait_s="$2"
  shift 2
  local pids=("$@")

  local pid
  for pid in "${pids[@]}"; do
    [[ -z "$pid" ]] && continue
    if pid_alive "$pid"; then
      kill -s "$sig" "$pid" >/dev/null 2>&1 || true
    fi
  done

  local elapsed=0
  while [[ $elapsed -lt $wait_s ]]; do
    local any=0
    for pid in "${pids[@]}"; do
      if pid_alive "$pid"; then
        any=1
        break
      fi
    done
    [[ $any -eq 0 ]] && break
    sleep 1
    elapsed=$((elapsed + 1))
  done
}

collect_descendants() {
  local -n _out=$1
  shift
  local queue=("$@")
  local -A seen=()
  _out=()

  while [[ ${#queue[@]} -gt 0 ]]; do
    local parent="${queue[0]}"
    queue=("${queue[@]:1}")
    [[ -z "$parent" ]] && continue
    if [[ -n "${seen[$parent]:-}" ]]; then
      continue
    fi
    seen[$parent]=1
    while IFS= read -r child; do
      [[ -z "$child" ]] && continue
      child="${child//[[:space:]]/}"
      [[ -z "$child" ]] && continue
      if [[ -z "${seen[$child]:-}" ]]; then
        queue+=("$child")
        _out+=("$child")
      fi
    done < <(ps -o pid= --no-headers --ppid "$parent" 2>/dev/null || true)
  done
}

read_ids() {
  local pattern="$1"
  local -n out_arr=$2
  shopt -s nullglob
  for f in $pattern; do
    val="$(tr -d ' \n\t' <"$f" || true)"
    if [[ "$val" =~ ^[0-9]+$ ]]; then
      out_arr+=("$val")
    fi
  done
  shopt -u nullglob
}

scan_orphan_launches() {
  local -n _out=$1
  _out=()
  # 匹配当前用户、命令行含 "ros2 launch"
  while IFS= read -r pid; do
    [[ -z "$pid" ]] && continue
    _out+=("$pid")
  done < <(pgrep -u "$USER" -f "ros2[[:space:]]+launch" 2>/dev/null || true)
}

scan_workspace_binaries() {
  local -n _out=$1
  _out=()
  local pattern="${COLCON_INSTALL:-$WS_ROOT/.colcon/install}/"
  while IFS= read -r pid; do
    [[ -z "$pid" ]] && continue
    _out+=("$pid")
  done < <(pgrep -u "$USER" -f "$pattern" 2>/dev/null || true)
}

# ----------------------------------------------------------------------------
# 收集 PGID / PID / TERM PID
# ----------------------------------------------------------------------------
PGIDS=()
LAUNCH_PIDS=()
SESSION_IDS=()
TERM_PIDS=()
DESC_PIDS=()
ORPHAN_PIDS=()
PIDFILE_DATA=0

read_ids "$TARGET_DIR/launch_*.pgid" PGIDS
read_ids "$TARGET_DIR/launch_*.pid" LAUNCH_PIDS
read_ids "$TARGET_DIR/launch_*.sid" SESSION_IDS
read_ids "$TARGET_DIR/term_*.pid" TERM_PIDS
if [[ "${#PGIDS[@]}" -gt 0 || "${#LAUNCH_PIDS[@]}" -gt 0 || "${#SESSION_IDS[@]}" -gt 0 || "${#TERM_PIDS[@]}" -gt 0 ]]; then
  PIDFILE_DATA=1
fi

# 补充扫描：即便有 pidfiles，也兜底清理工作区下的 ros2 launch / 安装产物进程
EXTRA_LAUNCH_PIDS=()
scan_orphan_launches EXTRA_LAUNCH_PIDS
FILTERED_EXTRA_LAUNCH_PIDS=()
for pid in "${EXTRA_LAUNCH_PIDS[@]}"; do
  pid_alive "$pid" || continue
  if pid_cwd_in_ws "$pid"; then
    LAUNCH_PIDS+=("$pid")
    FILTERED_EXTRA_LAUNCH_PIDS+=("$pid")
  fi
done

EXTRA_ORPHAN_PIDS=()
scan_workspace_binaries EXTRA_ORPHAN_PIDS
ORPHAN_PIDS+=("${EXTRA_ORPHAN_PIDS[@]}")

if [[ $PIDFILE_DATA -eq 0 && ( "${#LAUNCH_PIDS[@]}" -gt 0 || "${#ORPHAN_PIDS[@]}" -gt 0 ) ]]; then
  print_color yellow "No pidfiles; fallback to workspace scan (cwd under $WS_ROOT)."
fi

# 若依旧没有可清理的目标
if [[ "${#PGIDS[@]}" -eq 0 && "${#LAUNCH_PIDS[@]}" -eq 0 && "${#SESSION_IDS[@]}" -eq 0 && "${#ORPHAN_PIDS[@]}" -eq 0 ]]; then
  if [[ $KILL_RVIZ -eq 0 && "${#EXTRA_PATTERNS_ALWAYS[@]}" -eq 0 ]]; then
    print_color yellow "No launch pgid/pid/sid found; nothing to kill."
    exit 0
  else
    print_color yellow "No launch pgid/pid/sid found; will kill known patterns only."
  fi
fi

# 若没有 pgid，用 pid 推导
for pid in "${LAUNCH_PIDS[@]}"; do
  if kill -0 "$pid" >/dev/null 2>&1; then
    pgid="$(ps -o pgid= "$pid" | tr -d ' ' || true)"
    [[ "$pgid" =~ ^[0-9]+$ ]] && PGIDS+=("$pgid")
  fi
done

# 去重
if [[ "${#PGIDS[@]}" -gt 0 ]]; then
  PGIDS=($(printf "%s\n" "${PGIDS[@]}" | sort -u))
fi

if [[ "${#PGIDS[@]}" -eq 0 ]]; then
  warn "No valid PGIDs found. Will try PID/SID fallback; else only close terminals."
else
  print_color cyan "Target PGIDs: ${PGIDS[*]}"
  for pgid in "${PGIDS[@]}"; do
    list_group "$pgid"
  done
fi

if [[ "${#LAUNCH_PIDS[@]}" -gt 0 ]]; then
  LAUNCH_PIDS=($(printf "%s\n" "${LAUNCH_PIDS[@]}" | sort -u))
  print_color cyan "Target PIDs : ${LAUNCH_PIDS[*]}"
  for pid in "${LAUNCH_PIDS[@]}"; do
    ps -o pid=,pgid=,cmd= -p "$pid" 2>/dev/null | sed 's/^/  /' || true
  done
fi

if [[ "${#SESSION_IDS[@]}" -eq 0 && "${#LAUNCH_PIDS[@]}" -gt 0 ]]; then
  for pid in "${LAUNCH_PIDS[@]}"; do
    sid="$(ps -o sid= "$pid" | tr -d ' ' || true)"
    [[ "$sid" =~ ^[0-9]+$ ]] && SESSION_IDS+=("$sid")
  done
fi

if [[ "${#ORPHAN_PIDS[@]}" -gt 0 ]]; then
  ORPHAN_PIDS=($(printf "%s\n" "${ORPHAN_PIDS[@]}" | sort -u))
  print_color cyan "Workspace procs (.colcon/install/*): ${ORPHAN_PIDS[*]}"
  for pid in "${ORPHAN_PIDS[@]}"; do
    ps -o pid=,ppid=,pgid=,sid=,cmd= -p "$pid" 2>/dev/null | sed 's/^/  /' || true
  done
fi

if [[ "${#LAUNCH_PIDS[@]}" -gt 0 ]]; then
  collect_descendants DESC_PIDS "${LAUNCH_PIDS[@]}"
  if [[ "${#DESC_PIDS[@]}" -gt 0 ]]; then
    DESC_PIDS=($(printf "%s\n" "${DESC_PIDS[@]}" | sort -u))
    print_color cyan "Descendants : ${DESC_PIDS[*]}"
    for pid in "${DESC_PIDS[@]}"; do
      ps -o pid=,ppid=,pgid=,sid=,cmd= -p "$pid" 2>/dev/null | sed 's/^/  /' || true
    done
  fi
fi

if [[ "${#SESSION_IDS[@]}" -gt 0 ]]; then
  SESSION_IDS=($(printf "%s\n" "${SESSION_IDS[@]}" | sort -u))
  print_color cyan "Target SIDs : ${SESSION_IDS[*]}"
  for sid in "${SESSION_IDS[@]}"; do
    ps -o pid=,pgid=,sid=,cmd= --sid "$sid" 2>/dev/null | sed 's/^/  /' || true
  done
fi

if [[ $KILL_RVIZ -eq 1 ]]; then
  EXTRA_PATTERNS=(rviz2 foxglove_bridge)
  print_color cyan "Extra target patterns: ${EXTRA_PATTERNS[*]} (current user)"
  for pat in "${EXTRA_PATTERNS[@]}"; do
    pgrep -u "$USER" -af "$pat" 2>/dev/null | sed 's/^/  /' || true
  done
fi

if [[ $DRY_RUN -eq 1 ]]; then
  if [[ "${#TERM_PIDS[@]}" -gt 0 ]]; then
    print_color cyan "Dry-run: would close terminal PIDs: ${TERM_PIDS[*]}"
  fi
  if [[ $KILL_RVIZ -eq 1 ]]; then
    print_color cyan "Dry-run: would kill patterns '${EXTRA_PATTERNS[*]}' for user $USER"
  fi
  if [[ "${#DESC_PIDS[@]}" -gt 0 ]]; then
    print_color cyan "Dry-run: would kill descendants: ${DESC_PIDS[*]}"
  fi
  if [[ "${#ORPHAN_PIDS[@]}" -gt 0 ]]; then
    print_color cyan "Dry-run: would kill workspace procs: ${ORPHAN_PIDS[*]}"
  fi
  print_color cyan "Dry-run: no signals sent."
  exit 0
fi

if [[ "${#PGIDS[@]}" -gt 0 ]]; then
  print_color cyan "Sending INT -> TERM -> KILL ..."
  kill_group_stage INT 3 "${PGIDS[@]}"
  kill_group_stage TERM 2 "${PGIDS[@]}"
  kill_group_stage KILL 0 "${PGIDS[@]}"
fi

if [[ "${#LAUNCH_PIDS[@]}" -gt 0 ]]; then
  print_color cyan "Sending INT -> TERM -> KILL to PIDs ..."
  kill_pid_stage INT 3 "${LAUNCH_PIDS[@]}"
  kill_pid_stage TERM 2 "${LAUNCH_PIDS[@]}"
  kill_pid_stage KILL 0 "${LAUNCH_PIDS[@]}"
fi

if [[ "${#DESC_PIDS[@]}" -gt 0 ]]; then
  print_color cyan "Sending INT -> TERM -> KILL to descendants ..."
  kill_pid_stage INT 3 "${DESC_PIDS[@]}"
  kill_pid_stage TERM 2 "${DESC_PIDS[@]}"
  kill_pid_stage KILL 0 "${DESC_PIDS[@]}"
fi

if [[ "${#ORPHAN_PIDS[@]}" -gt 0 ]]; then
  print_color cyan "Sending INT -> TERM -> KILL to workspace procs ..."
  kill_pid_stage INT 3 "${ORPHAN_PIDS[@]}"
  kill_pid_stage TERM 2 "${ORPHAN_PIDS[@]}"
  kill_pid_stage KILL 0 "${ORPHAN_PIDS[@]}"
fi

if [[ "${#SESSION_IDS[@]}" -gt 0 ]]; then
  print_color cyan "Sending INT -> TERM -> KILL to SIDs ..."
  kill_sid_stage INT 3 "${SESSION_IDS[@]}"
  kill_sid_stage TERM 2 "${SESSION_IDS[@]}"
  kill_sid_stage KILL 0 "${SESSION_IDS[@]}"
fi

if [[ $KILL_RVIZ -eq 1 ]]; then
  print_color cyan "Sending INT -> TERM -> KILL to patterns (${EXTRA_PATTERNS[*]}) user=$USER ..."
  for pat in "${EXTRA_PATTERNS[@]}"; do
    kill_pattern_stage INT 3 "$pat"
    kill_pattern_stage TERM 2 "$pat"
    kill_pattern_stage KILL 0 "$pat"
  done
fi

if [[ "${#EXTRA_PATTERNS_ALWAYS[@]}" -gt 0 ]]; then
  print_color cyan "Sending INT -> TERM -> KILL to common ROS patterns (${EXTRA_PATTERNS_ALWAYS[*]}) user=$USER ..."
  for pat in "${EXTRA_PATTERNS_ALWAYS[@]}"; do
    kill_pattern_stage INT 2 "$pat"
    kill_pattern_stage TERM 2 "$pat"
    kill_pattern_stage KILL 0 "$pat"
  done
fi

if [[ $CLOSE_TERM -eq 1 && "${#TERM_PIDS[@]}" -gt 0 ]]; then
  print_color cyan "Closing terminal shells (INT -> TERM -> KILL) ..."
  kill_pid_stage INT 1 "${TERM_PIDS[@]}"
  kill_pid_stage TERM 1 "${TERM_PIDS[@]}"
  kill_pid_stage KILL 0 "${TERM_PIDS[@]}"
fi

LEFT=0
for pgid in "${PGIDS[@]}"; do
  if group_alive "$pgid"; then
    LEFT=1
    print_color red "Still alive: PGID=$pgid"
    list_group "$pgid"
  fi
done
for pid in "${LAUNCH_PIDS[@]}"; do
  if pid_alive "$pid"; then
    LEFT=1
    print_color red "Still alive: PID=$pid"
    ps -o pid=,pgid=,sid=,cmd= -p "$pid" 2>/dev/null | sed 's/^/  /' || true
  fi
done
for pid in "${DESC_PIDS[@]}"; do
  if pid_alive "$pid"; then
    LEFT=1
    print_color red "Still alive: DESC_PID=$pid"
    ps -o pid=,ppid=,pgid=,sid=,cmd= -p "$pid" 2>/dev/null | sed 's/^/  /' || true
  fi
done
for pid in "${ORPHAN_PIDS[@]}"; do
  if pid_alive "$pid"; then
    LEFT=1
    print_color red "Still alive: WS_PID=$pid"
    ps -o pid=,ppid=,pgid=,sid=,cmd= -p "$pid" 2>/dev/null | sed 's/^/  /' || true
  fi
done

for sid in "${SESSION_IDS[@]}"; do
  if session_alive "$sid"; then
    LEFT=1
    print_color red "Still alive: SID=$sid"
    ps -o pid=,pgid=,sid=,cmd= --sid "$sid" 2>/dev/null | sed 's/^/  /' || true
  fi
done

if [[ $KILL_RVIZ -eq 1 ]]; then
  for pat in "${EXTRA_PATTERNS[@]}"; do
    if pattern_alive "$pat"; then
      LEFT=1
      print_color red "Still alive: pattern $pat"
      pgrep -u "$USER" -af "$pat" 2>/dev/null | sed 's/^/  /' || true
    fi
  done
fi

if [[ $LEFT -eq 0 ]]; then
  if [[ "${#PGIDS[@]}" -gt 0 ]]; then
    print_color green "All target process groups terminated."
  else
    print_color yellow "No process groups to kill."
  fi
  if [[ $KEEP_DIR -eq 0 ]]; then
    if [[ -n "$TARGET_DIR" ]]; then
      rm -rf "$TARGET_DIR"
      print_color yellow "Removed RUN_DIR: $TARGET_DIR"
    fi
    if [[ $USE_LATEST -eq 1 && -f "$LATEST_FILE" ]]; then
      rm -f "$LATEST_FILE"
      print_color yellow "Cleared LATEST pointer."
    fi
  else
    print_color yellow "KEEP enabled: RUN_DIR retained at $TARGET_DIR"
  fi
else
  print_color red "Some processes survived. RUN_DIR kept at: $TARGET_DIR"
  exit 1
fi

# 刷新 ros2 CLI 守护进程，避免残留列表缓存
if command -v ros2 >/dev/null 2>&1; then
  ros2 daemon stop >/dev/null 2>&1 || true
fi
