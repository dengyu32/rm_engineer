#!/bin/bash
# ==============================================================================
# 脚本名称: gen_compile_commands.sh
# 描述: 合并所有 ROS 2 软件包的编译数据库，并为头文件生成合成条目。
#       主要用于提升 clangd 对大型 ROS 2 工作空间的索引能力。
# ==============================================================================

set -euo pipefail

# ------------------------------------------------------------------------------
# 1. 环境初始化
# ------------------------------------------------------------------------------
SCRIPT_FILE="$(realpath "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$SCRIPT_FILE")"
SHLIB_DIR="$SCRIPT_DIR"

# shellcheck source=common.sh
source "$SHLIB_DIR/common.sh"
# shellcheck source=paths.sh
source "$SHLIB_DIR/paths.sh"

require_cmd python3 realpath

# 定义路径，优先使用环境变量
ROOT="$WS_ROOT"
BUILD_DIR="${COLCON_BUILD:-$ROOT/.colcon/build}"
OUT_FILE="$ROOT/compile_commands.json"

if [[ ! -d "$BUILD_DIR" ]]; then
    print_color yellow "[clangd] 构建目录不存在: $BUILD_DIR (跳过合并)"
    exit 0
fi

cd "$ROOT"

# ------------------------------------------------------------------------------
# 2. Python 处理逻辑
# ------------------------------------------------------------------------------
# 使用 Python 脚本处理复杂的 JSON 合并与头文件推导
python3 - <<'PY'
import json
import pathlib
import os
import shlex
import sys

def log(msg):
    print(f"[clangd] {msg}")

root = pathlib.Path.cwd()
build_dir = pathlib.Path(os.environ.get("COLCON_BUILD", root / ".colcon" / "build"))
install_dir = pathlib.Path(os.environ.get("COLCON_INSTALL", root / ".colcon" / "install"))

# 1. 基础合并逻辑
entries = []
log(f"正在扫描编译数据库: {build_dir}")

for path in build_dir.rglob("compile_commands.json"):
    try:
        with open(path, 'r') as f:
            data = json.load(f)
        for item in data:
            # 转换为绝对路径，确保 clangd 在任何目录下都能正确识别
            directory = pathlib.Path(item.get("directory", ".")).resolve()
            file_path = (directory / item.get("file", "")).resolve()
            item["directory"] = str(directory)
            item["file"] = str(file_path)
            entries.append(item)
    except Exception as e:
        log(f"跳过损坏的文件 {path}: {e}")

if not entries:
    log("未找到任何编译条目。")
    sys.exit(0)

# 2. 收集 Include 路径 (用于合成头文件命令)
def gather_include_dirs(root_path, cdb_entries):
    inc = set()
    
    # 从现有的编译命令中提取路径 (最准确)
    for e in cdb_entries:
        cmd = e.get("command", "")
        if not cmd: continue
        try:
            parts = shlex.split(cmd)
            for i, p in enumerate(parts):
                if p in ("-I", "-isystem") and i + 1 < len(parts):
                    inc.add(str(pathlib.Path(parts[i+1]).resolve()))
                elif p.startswith("-I"):
                    inc.add(str(pathlib.Path(p[2:]).resolve()))
        except: continue

    # 扫描本地源码目录
    src_dir = root_path / "src"
    if src_dir.exists():
        for p in src_dir.rglob("include"):
            if p.is_dir(): inc.add(str(p.resolve()))

    # 扫描安装目录
    if install_dir.exists():
        for p in install_dir.rglob("include"):
            if p.is_dir(): inc.add(str(p.resolve()))

    # 扫描 ROS 环境
    ament_paths = os.environ.get("AMENT_PREFIX_PATH", "").split(":")
    for prefix in filter(None, ament_paths):
        inc_dir = pathlib.Path(prefix) / "include"
        if inc_dir.exists(): inc.add(str(inc_dir.resolve()))
        
    return sorted(list(inc))

# 3. 为“孤儿”头文件生成条目
include_dirs = gather_include_dirs(root, entries)
existing_files = {e.get("file") for e in entries if "file" in e}
added_count = 0

log("正在为头文件生成合成条目...")
# 遍历 src 下的所有 C++ 头文件
header_patterns = ["**/*.h", "**/*.hh", "**/*.hpp", "**/*.hxx"]
for pattern in header_patterns:
    for h in (root / "src").rglob(pattern):
        h_str = str(h.resolve())
        if h_str in existing_files:
            continue

        # 构造基础编译命令，让 clangd 知道如何解析这个头文件
        cmd_parts = ["/usr/bin/g++", "-std=c++17", "-x", "c++-header"]
        for d in include_dirs:
            # 区分系统头文件和普通头文件，优化警告处理
            flag = "-isystem" if "opt/ros" in d or "third_party" in d else "-I"
            cmd_parts.extend([flag, d])
        
        cmd_parts.extend(["-c", h_str])
        
        entries.append({
            "directory": str(root),
            "command": " ".join(cmd_parts),
            "file": h_str
        })
        added_count += 1

# 4. 写入结果
out_path = root / "compile_commands.json"
with open(out_path, 'w') as f:
    json.dump(entries, f, indent=2)

log(f"完成！总计条目: {len(entries)} (新增头文件: {added_count})")
PY