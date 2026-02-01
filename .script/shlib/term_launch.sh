#!/bin/bash
set -euo pipefail

# ----------------------------------------------------------------------------
# term_launch.sh : open_term helper
# ----------------------------------------------------------------------------

# shellcheck source=common.sh
# shellcheck source=logging.sh
# shellcheck source=paths.sh
SHLIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SHLIB_DIR/common.sh"
source "$SHLIB_DIR/logging.sh"
source "$SHLIB_DIR/paths.sh"

open_term() {
  local title="${1:?title required}"
  local cmd="${2:?cmd required}"
  local ros_setup="${3:?ROS setup required}"
  local ws_setup="${4:?WS setup required}"
  local run_dir="${5:?RUN_DIR required}"
  local log_base="${6:?LOG_BASE required}"

  require_cmd gnome-terminal tee mkfifo setsid ps

  local safe_title
  safe_title="$(sanitize_title "$title")"

  local launch_pid_file="$run_dir/launch_${safe_title}.pid"
  local launch_pgid_file="$run_dir/launch_${safe_title}.pgid"
  local launch_sid_file="$run_dir/launch_${safe_title}.sid"
  local term_pid_file="$run_dir/term_${safe_title}.pid"
  local fifo_file="$run_dir/pipe_${safe_title}.fifo"
  local runner_file="$run_dir/runner_${safe_title}.sh"
  local base_log_file="$log_base/${safe_title}.log"
  local run_log_file="$run_dir/${safe_title}.log"

  mkdir -p "$run_dir" "$log_base"

  log_header "$base_log_file" "$title" "$cmd"
  log_header "$run_log_file" "$title" "$cmd"

  cat >"$runner_file" <<EOF
#!/bin/bash
set -euo pipefail
set -m
source "$SHLIB_DIR/common.sh"
source "$SHLIB_DIR/logging.sh"
set +u
source "$ros_setup"
source "$ws_setup"
set -u

print_color cyan "$title"
echo \$\$ >"$term_pid_file"

fifo="$fifo_file"
[[ -p "\$fifo" ]] || mkfifo "\$fifo"
tee -a "$base_log_file" "$run_log_file" <"\$fifo" &
tee_pid=\$!

setsid bash -c '$cmd' >"\$fifo" 2>&1 &
launch_pid=\$!
pgid=\$(ps --no-headers -o pgid= -p "\$launch_pid" 2>/dev/null | tr -d ' ' || true)
if [[ -z "\$pgid" ]]; then
  pgid="\$launch_pid"
fi
sid=\$(ps --no-headers -o sid= -p "\$launch_pid" 2>/dev/null | tr -d ' ' || true)
if [[ -z "\$sid" ]]; then
  sid="\$launch_pid"
fi
echo \$launch_pid >"$launch_pid_file"
echo \$pgid >"$launch_pgid_file"
echo \$sid >"$launch_sid_file"

set +e
wait "\$launch_pid"
exit_code=\$?
set -e

log_footer "$base_log_file" "\$exit_code"
log_footer "$run_log_file" "\$exit_code"

exec </dev/null
wait "\$tee_pid" >/dev/null 2>&1 || true
rm -f "\$fifo"
print_color yellow "[${title}] launch exited with code \$exit_code"
exec bash
EOF

  chmod +x "$runner_file"
  gnome-terminal --window --title="$title" -- bash -ic "bash '$runner_file'"
}
