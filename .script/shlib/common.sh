#!/bin/bash
# ==============================================================================
# 脚本名称: common.sh
# 描述: 共享辅助工具库，提供颜色打印、错误处理、命令检查等基础功能。
# ==============================================================================

set -euo pipefail

# ------------------------------------------------------------------------------
# 1. 终端颜色输出
# ------------------------------------------------------------------------------

# 函数: print_color
# 描述: 向终端输出带颜色的文本
# 参数: $1 - 颜色名称 (green, red, yellow, blue, purple, cyan, white)
#       $2 - 要显示的文本内容
print_color() {
    local color="${1:-}"
    local text="${2:-}"
    local color_code

    case "$color" in
        green)  color_code=32 ;;
        red)    color_code=31 ;;
        yellow) color_code=33 ;;
        blue)   color_code=34 ;;
        purple) color_code=35 ;;
        cyan)   color_code=36 ;;
        white)  color_code=37 ;;
        *)      color_code=0  ;;
    esac
    
    echo -e "\e[${color_code}m${text}\e[0m"
}

# 快捷日志函数
die()  { print_color red    "[FATAL] ${1:-未知错误}"; exit 1; }
warn() { print_color yellow "[WARN]  ${1:-警告}"; }
info() { print_color cyan   "[INFO]  ${1:-信息}"; }

# ------------------------------------------------------------------------------
# 2. 系统环境检查
# ------------------------------------------------------------------------------

# 函数: require_cmd
# 描述: 检查系统是否安装了必要的命令，若缺失则退出脚本
# 参数: $@ - 命令名称列表 (例如: require_cmd ros2 colcon git)
require_cmd() {
    local missing=0
    local required_cmd
    for required_cmd in "$@"; do
        if ! command -v "$required_cmd" >/dev/null 2>&1; then
            warn "缺失必要命令: $required_cmd"
            missing=1
        fi
    done
    
    [[ $missing -eq 0 ]] || die "由于缺少上述必要组件，脚本无法运行。"
}

# ------------------------------------------------------------------------------
# 3. 路径与字符串处理
# ------------------------------------------------------------------------------

# 函数: sanitize_title
# 描述: 清理字符串，使其适合作为文件名或标识符
# 参数: $1 - 原始标题字符串
# 逻辑: 将空格和斜杠替换为下划线，移除特殊字符
sanitize_title() {
    local input="${1:-}"
    echo "$input" | tr ' /' '__' | tr -cd 'A-Za-z0-9_.-'
}

# 函数: get_script_dir
# 描述: 获取当前脚本所在的绝对路径（支持符号链接追踪）
# 返回: 路径字符串
get_script_dir() {
    local src="${BASH_SOURCE[0]}"
    while [ -h "$src" ]; do
        local dir
        dir="$(cd -P "$(dirname "$src")" && pwd)"
        src="$(readlink "$src")"
        [[ $src != /* ]] && src="$dir/$src"
    done
    cd -P "$(dirname "$src")" && pwd
}

# ------------------------------------------------------------------------------
# 4. 安全防护
# ------------------------------------------------------------------------------

# 函数: realpath_guard_prefix
# 描述: 路径溢出防护。确保目标路径在指定的基准目录之内。
# 参数: $1 - 目标路径 (Target Path)
#       $2 - 基准目录 (Base Directory)
# 目的: 防止由于变量错误或恶意输入导致 rm -rf 等命令操作到系统根目录
realpath_guard_prefix() {
    local target="${1:?未指定目标路径}"
    local base="${2:?未指定基准目录}"
    
    local real_base real_target
    real_base="$(realpath "$base")"
    real_target="$(realpath "$target")"

    # 仅允许 base 自身或其真实子目录，避免 /foo/bar 与 /foo/bar_bak 这类前缀误判。
    if [[ "$real_target" == "$real_base" || "$real_target" == "$real_base/"* ]]; then
        return 0
    else
        die "安全拒绝: 目标路径 [$real_target] 超出基准范围 [$real_base]"
    fi
}
