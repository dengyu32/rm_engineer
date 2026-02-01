#!/bin/bash
set -euo pipefail

# ----------------------------------------------------------------------------
# common.sh : shared helpers for rerun tooling
# ----------------------------------------------------------------------------

print_color() {
  local color_code
  case "${1:-}" in
    green)  color_code=32 ;;
    red)    color_code=31 ;;
    yellow) color_code=33 ;;
    blue)   color_code=34 ;;
    purple) color_code=35 ;;
    cyan)   color_code=36 ;;
    white)  color_code=37 ;;
    *)      color_code=0  ;;
  esac
  echo -e "\e[${color_code}m${2:-}\e[0m"
}

die()  { print_color red   "${1:-fatal}"; exit 1; }
warn() { print_color yellow "${1:-warn}"; }
info() { print_color cyan  "${1:-info}"; }

require_cmd() {
  local missing=0
  for c in "$@"; do
    if ! command -v "$c" >/dev/null 2>&1; then
      warn "Missing command: $c"
      missing=1
    fi
  done
  [[ $missing -eq 0 ]] || die "Required command(s) missing."
}

sanitize_title() {
  # 保留简洁字符，空格和斜杠替换为下划线，适合文件名
  echo "${1:-}" | tr ' /' '__' | tr -cd 'A-Za-z0-9_.-'
}

script_dir() {
  local src="${BASH_SOURCE[0]}"
  while [ -h "$src" ]; do
    local dir
    dir="$(cd -P "$(dirname "$src")" && pwd)"
    src="$(readlink "$src")"
    [[ $src != /* ]] && src="$dir/$src"
  done
  cd -P "$(dirname "$src")" && pwd
}

realpath_guard_prefix() {
  local target="${1:?target path required}"
  local base="${2:?base path required}"
  local real_base real_target
  real_base="$(realpath "$base")"
  real_target="$(realpath "$target")"

  case "$real_target" in
    "$real_base"/*) return 0 ;;
    "$real_base")   return 0 ;;
    *)
      die "Refuse to operate outside base: target=$real_target base=$real_base"
      ;;
  esac
}
