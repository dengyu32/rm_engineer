#!/bin/bash
set -euo pipefail

# ----------------------------------------------------------------------------
# logging.sh : timestamped log helpers
# ----------------------------------------------------------------------------
now_human() {
  date +"%Y-%m-%d %H:%M:%S"
}

log_header() {
  local file="${1:?file required}"
  local title="${2:-}"
  local cmd="${3:-}"
  {
    echo "===== START $(now_human) ====="
    [[ -n "$title" ]] && echo "TITLE: $title"
    [[ -n "$cmd" ]] && echo "CMD  : $cmd"
    echo "--------------------------------"
  } >> "$file"
}

log_footer() {
  local file="${1:?file required}"
  local ret="${2:-0}"
  {
    echo "--------------------------------"
    echo "RET  : $ret"
    echo "END  : $(now_human)"
    echo "===== END ====="
    echo
  } >> "$file"
}
