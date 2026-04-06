#!/bin/bash
# ==============================================================================
# 脚本名称: check_env.sh
# 描述: 自动检查并准备构建依赖。包括 ONNX Runtime 的下载解压，
#       以及 RealSense SDK (librealsense) 的源码下载与本地编译。
# ==============================================================================

set -euo pipefail

# ------------------------------------------------------------------------------
# 1. 环境加载与辅助工具
# ------------------------------------------------------------------------------
SCRIPT_FILE="$(realpath "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$SCRIPT_FILE")"
SHLIB_DIR="$SCRIPT_DIR"

# shellcheck source=common.sh
source "$SHLIB_DIR/common.sh"
# shellcheck source=paths.sh
source "$SHLIB_DIR/paths.sh"

require_cmd realpath

# --- 内部辅助函数 ---

# 功能: 封装带有重试机制的 curl 下载
curl_download() {
    local url="${1:?url required}"
    local out="${2:?output required}"
    local -a curl_args=()
    rm -f "$out"

    [[ "${CURL_HTTP1_ONLY:-0}" == "1" ]] && curl_args+=(--http1.1)

    # 检查 curl 版本是否支持 --retry-all-errors
    if curl --help all 2>/dev/null | grep -q "retry-all-errors"; then
        curl -L --fail --retry 5 --retry-delay 2 --retry-all-errors "${curl_args[@]}" -o "$out" "$url"
    else
        curl -L --fail --retry 5 --retry-delay 2 "${curl_args[@]}" -o "$out" "$url"
    fi
}

# 功能: 多 URL 备选下载
download_with_fallback() {
    local out="${1:?output required}"
    local url
    shift
    for url in "$@"; do
        print_color cyan "正在尝试下载: $url"
        if curl_download "$url" "$out"; then return 0; fi
    done
    return 1
}

# 功能: 获取相对于工作空间的路径（用于整洁打印）
relpath_ws() {
    local target="${1:?target required}"
    realpath --relative-to="$WS_ROOT" "$target" 2>/dev/null || echo "$target"
}

# ------------------------------------------------------------------------------
# 2. ONNX Runtime 管理 (二进制分发版)
# ------------------------------------------------------------------------------
onnxruntime_paths() {
    local ort_version="${ONNX_RUNTIME_VERSION:-1.20.1}"
    ONNX_ORT_VERSION="$ort_version"
    ONNX_ORT_REL_DIR="src/capabilities/vision/YOLOs-CPP/onnxruntime-linux-x64-${ort_version}"
    ONNX_ORT_DIR="$WS_ROOT/$ONNX_ORT_REL_DIR"
    ONNX_ORT_HEADER="$ONNX_ORT_DIR/include/onnxruntime_cxx_api.h"
    ONNX_ORT_ARCHIVE="onnxruntime-linux-x64-${ort_version}.tgz"
    ONNX_ORT_URL="${ONNX_RUNTIME_URL:-https://github.com/microsoft/onnxruntime/releases/download/v${ort_version}/${ONNX_ORT_ARCHIVE}}"
    ONNX_ORT_ARCHIVE_PATH="$WS_ROOT/src/capabilities/vision/YOLOs-CPP/${ONNX_ORT_ARCHIVE}"
}

verify_onnxruntime() {
    onnxruntime_paths
    [[ -f "$ONNX_ORT_HEADER" ]]
}

ensure_onnxruntime() {
    onnxruntime_paths

    # 检查是否已存在
    if verify_onnxruntime; then
        print_color green "ONNX Runtime 已就绪: $(relpath_ws "$ONNX_ORT_DIR")"
        return 0
    fi

    require_cmd curl tar
    print_color yellow "未发现 ONNX Runtime，准备下载 v${ONNX_ORT_VERSION} ..."
    
    mkdir -p "$(dirname "$ONNX_ORT_DIR")"
    if ! curl_download "$ONNX_ORT_URL" "$ONNX_ORT_ARCHIVE_PATH"; then
        die "无法从 $ONNX_ORT_URL 下载 ONNX Runtime"
    fi

    print_color cyan "正在解压 ONNX Runtime..."
    tar -xzf "$ONNX_ORT_ARCHIVE_PATH" -C "$(dirname "$ONNX_ORT_DIR")"
    rm -f "$ONNX_ORT_ARCHIVE_PATH"

    [[ -f "$ONNX_ORT_HEADER" ]] || die "解压完成但未找到头文件: $ONNX_ORT_HEADER"
    print_color green "ONNX Runtime 准备完毕。"
}

# ------------------------------------------------------------------------------
# 3. RealSense SDK 管理 (源码编译版)
# ------------------------------------------------------------------------------
find_realsense_config() {
    local -a candidates=()
    local cfg
    # 优先使用工作区本地安装，再考虑显式环境变量和系统标准路径
    candidates+=(
        "$WS_ROOT/third_party/realsense-ros/librealsense_sdk/install/lib/cmake/realsense2/realsense2Config.cmake"
    )
    [[ -n "${realsense2_DIR:-}" ]] && candidates+=("$realsense2_DIR/realsense2Config.cmake")
    candidates+=(
        "/usr/lib/x86_64-linux-gnu/cmake/realsense2/realsense2Config.cmake"
        "/usr/local/lib/cmake/realsense2/realsense2Config.cmake"
    )

    for cfg in "${candidates[@]}"; do
        [[ -f "$cfg" ]] && { echo "$cfg"; return 0; }
    done
    return 1
}

verify_realsense_sdk() {
    local cfg
    if cfg="$(find_realsense_config)"; then
        export realsense2_DIR="$(dirname "$cfg")"
        return 0
    fi
    return 1
}

ensure_realsense_sdk() {
    local rs_version="${REALSENSE_VERSION:-2.57.5}"
    local rs_rel_root="${REALSENSE_ROOT_REL:-third_party/realsense-ros/librealsense_sdk}"
    local rs_root="$WS_ROOT/$rs_rel_root"
    local rs_install="$rs_root/install"
    
    # 策略: 优先复用已存在的本地或系统 RealSense SDK
    if verify_realsense_sdk; then
        print_color green "发现 RealSense SDK: $(relpath_ws "$realsense2_DIR")"
        return 0
    fi

    # 如果没有系统安装，则进行本地下载与编译
    require_cmd curl tar cmake
    print_color yellow "未发现 RealSense SDK，准备源码构建 v${rs_version} ..."
    
    # --- 下载源码 ---
    if [[ ! -f "$rs_root/CMakeLists.txt" ]]; then
        local rs_archive_path="$(dirname "$rs_root")/librealsense.tar.gz"
        local rs_url="https://github.com/IntelRealSense/librealsense/archive/refs/tags/v${rs_version}.tar.gz"
        
        download_with_fallback "$rs_archive_path" "$rs_url" || die "源码下载失败"
        
        mkdir -p "$rs_root"
        tar -xzf "$rs_archive_path" -C "$(dirname "$rs_root")"
        mv "$(dirname "$rs_root")/librealsense-${rs_version}"/* "$rs_root/"
        rm -f "$rs_archive_path"
    fi

    # --- CMake 编译与本地安装 ---
    print_color cyan "开始编译 RealSense SDK (本地模式)..."
    cmake -S "$rs_root" -B "$rs_root/build" \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$rs_install" \
        -DLRS_RUN_LDCONFIG=OFF \
        -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF \
        -DBUILD_PYTHON_BINDINGS=OFF -DBUILD_WITH_CUDA=OFF

    cmake --build "$rs_root/build" -j"$(nproc)"
    cmake --install "$rs_root/build"

    # 设置环境变量供后续 Colcon 编译使用
    export realsense2_DIR="$rs_install/lib/cmake/realsense2"
    export CMAKE_PREFIX_PATH="$rs_install${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
    
    # 关键：在本地 SDK 目录放置 COLCON_IGNORE，防止 Colcon 重复扫描
    touch "$rs_root/COLCON_IGNORE"
    print_color green "RealSense SDK 本地构建完成。"
}

# ------------------------------------------------------------------------------
# 4. 主入口
# ------------------------------------------------------------------------------
check_env() {
    local mode="${1:-prepare}"

    case "$mode" in
        verify)
            verify_onnxruntime || die "缺少 ONNX Runtime: $(relpath_ws "$WS_ROOT/src/capabilities/vision/YOLOs-CPP")"
            print_color green "ONNX Runtime 已就绪: $(relpath_ws "$ONNX_ORT_DIR")"

            verify_realsense_sdk || die "缺少 RealSense SDK 配置，请先执行一次构建路径完成依赖准备。"
            print_color green "发现 RealSense SDK: $(relpath_ws "$realsense2_DIR")"
            ;;
        prepare)
            ensure_onnxruntime
            ensure_realsense_sdk
            ;;
        *)
            die "未知 check_env 模式: $mode"
            ;;
    esac
}

# 允许脚本被 source 或直接执行
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    check_env
fi
