#!/bin/bash
set -euo pipefail

# 合并各包的 compile_commands.json 到工作区根目录，供 clangd 使用。

SCRIPT_FILE="$(realpath "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$SCRIPT_FILE")"
SHLIB_DIR="$SCRIPT_DIR/shlib"
source "$SHLIB_DIR/common.sh"
source "$SHLIB_DIR/paths.sh"

require_cmd python3 realpath

ROOT="$WS_ROOT"
BUILD_DIR="${COLCON_BUILD:-$ROOT/.colcon/build}"
OUT="$ROOT/compile_commands.json"

if [[ ! -d "$BUILD_DIR" ]]; then
  echo "[clangd] build directory not found: $BUILD_DIR (skip)"
  exit 0
fi

cd "$ROOT"

python3 - <<'PY'
import json
import pathlib
import os
import shlex

root = pathlib.Path.cwd()
build = pathlib.Path(os.environ.get("COLCON_BUILD", root / ".colcon" / "build"))
entries = []

for path in build.rglob("compile_commands.json"):
    try:
        data = json.loads(path.read_text())
    except Exception as e:
        print(f"[clangd] skip {path}: {e}")
        continue
    for item in data:
        directory = pathlib.Path(item.get("directory", ".")).resolve()
        file_path = (directory / item.get("file", "")).resolve()
        item["directory"] = str(directory)
        item["file"] = str(file_path)
        entries.append(item)

out = root / "compile_commands.json"
out.write_text(json.dumps(entries, indent=2))
print(f"[clangd] wrote {out} with {len(entries)} entries")

# 为没有 TU 的头文件补一条“合成编译命令”（适配 header-only 包）。
def gather_include_dirs(root_path: pathlib.Path, cdb_entries: list[dict]) -> list[str]:
    inc = set()
    # 复用真实编译命令里已有的 include 路径，避免硬编码导致路径不全/冲突。
    for e in cdb_entries:
        cmd = e.get("command")
        if not cmd:
            continue
        try:
            parts = shlex.split(cmd)
        except Exception:
            continue
        i = 0
        while i < len(parts):
            p = parts[i]
            if p in ("-I", "-isystem") and i + 1 < len(parts):
                inc.add(parts[i + 1])
                i += 2
                continue
            if p.startswith("-I") and len(p) > 2:
                inc.add(p[2:])
            elif p.startswith("-isystem") and len(p) > 8:
                inc.add(p[8:])
            i += 1
    # 工作区源码 include 目录（src/**/include）。
    for p in (root_path / "src").rglob("include"):
        if p.is_dir():
            inc.add(str(p.resolve()))
    # 工作区安装 include 目录（默认 .colcon/install，可被 COLCON_INSTALL 覆盖）。
    install = pathlib.Path(os.environ.get("COLCON_INSTALL", root_path / ".colcon" / "install"))
    if install.exists():
        for p in install.rglob("include"):
            if p.is_dir():
                inc.add(str(p.resolve()))
    # ROS 2 环境 include 目录（AMENT_PREFIX_PATH 可能包含多个前缀）。
    ament_prefix = os.environ.get("AMENT_PREFIX_PATH", "")
    for prefix in [p for p in ament_prefix.split(":") if p]:
        inc_dir = pathlib.Path(prefix) / "include"
        if inc_dir.exists():
            inc.add(str(inc_dir.resolve()))
    return sorted(inc)

include_dirs = gather_include_dirs(root, entries)
existing_files = {e.get("file") for e in entries if "file" in e}
headers = []
for pattern in ("*.h", "*.hh", "*.hpp", "*.hxx"):
    headers.extend((root / "src").rglob(pattern))

added = 0
for h in headers:
    h_resolved = str(h.resolve())
    if h_resolved in existing_files:
        continue
    # 统一为 header 构造一条可用的编译命令，供 clangd 解析。
    cmd_parts = ["/usr/bin/g++", "-std=gnu++17", "-x", "c++"]
    for inc in include_dirs:
        cmd_parts.extend(["-isystem" if inc == "/opt/ros/humble/include" else "-I", inc])
    cmd_parts.extend(["-c", h_resolved])
    entries.append({
        "directory": str(root.resolve()),
        "command": " ".join(cmd_parts),
        "file": h_resolved,
    })
    added += 1

if added:
    out.write_text(json.dumps(entries, indent=2))
    print(f"[clangd] added {added} synthetic header entries")
PY
