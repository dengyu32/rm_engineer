#!/bin/bash
set -euo pipefail

# ----------------------------------------------------------------------------
# check_env.sh : ensure local deps for build (ONNX Runtime / RealSense SDK)
# ----------------------------------------------------------------------------

SCRIPT_FILE="$(realpath "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$SCRIPT_FILE")"
SHLIB_DIR="$SCRIPT_DIR/shlib"
source "$SHLIB_DIR/common.sh"
source "$SHLIB_DIR/paths.sh"
source "$SHLIB_DIR/logging.sh"

require_cmd realpath

curl_download() {
  local url="${1:?url required}"
  local out="${2:?output required}"
  local -a curl_args=()
  rm -f "$out"
  if [[ "${CURL_HTTP1_ONLY:-0}" == "1" ]]; then
    curl_args+=(--http1.1)
  fi
  if curl --help all 2>/dev/null | grep -q "retry-all-errors"; then
    curl -L --fail --retry 5 --retry-delay 2 --retry-all-errors "${curl_args[@]}" -o "$out" "$url"
  else
    curl -L --fail --retry 5 --retry-delay 2 "${curl_args[@]}" -o "$out" "$url"
  fi
}

download_with_fallback() {
  local out="${1:?output required}"
  shift
  local url
  for url in "$@"; do
    if curl_download "$url" "$out"; then
      return 0
    fi
  done
  return 1
}

relpath_ws() {
  local target="${1:?target required}"
  realpath --relative-to="$WS_ROOT" "$target" 2>/dev/null || echo "$target"
}

ensure_onnxruntime() {
  local ort_version="${ONNX_RUNTIME_VERSION:-1.20.1}"
  local ort_rel_dir="src/vision/YOLOs-CPP/onnxruntime-linux-x64-${ort_version}"
  local ort_dir="$WS_ROOT/$ort_rel_dir"
  local ort_header="$ort_dir/include/onnxruntime_cxx_api.h"
  local ort_archive="onnxruntime-linux-x64-${ort_version}.tgz"
  local ort_url="${ONNX_RUNTIME_URL:-https://github.com/microsoft/onnxruntime/releases/download/v${ort_version}/${ort_archive}}"
  local ort_archive_path="$WS_ROOT/src/vision/YOLOs-CPP/${ort_archive}"

  if [[ -f "$ort_header" ]]; then
    print_color green "ONNX Runtime ready: $(relpath_ws "$ort_dir")"
    return 0
  fi

  require_cmd curl tar
  print_color yellow "ONNX Runtime missing, downloading v${ort_version} ..."
  print_color yellow "Source: $ort_url"

  rm -f "$ort_archive_path"
  if ! curl_download "$ort_archive_path" "$ort_url"; then
    print_color red "Failed to download ONNX Runtime from: $ort_url"
    return 1
  fi

  rm -rf "$ort_dir"
  if ! tar -xzf "$ort_archive_path" -C "$WS_ROOT/src/vision/YOLOs-CPP"; then
    print_color red "Failed to extract ONNX Runtime archive: $ort_archive_path"
    return 1
  fi
  rm -f "$ort_archive_path"

  if [[ ! -f "$ort_header" ]]; then
    print_color red "ONNX Runtime extraction completed but header not found: $ort_header"
    return 1
  fi

  print_color green "ONNX Runtime prepared: $(relpath_ws "$ort_dir")"
}

find_realsense_config() {
  local cfg=""
  local -a candidates=()

  if [[ -n "${realsense2_DIR:-}" ]]; then
    candidates+=("$realsense2_DIR/realsense2Config.cmake")
    candidates+=("$realsense2_DIR/realsense2-config.cmake")
    candidates+=("$realsense2_DIR/lib/cmake/realsense2/realsense2Config.cmake")
    candidates+=("$realsense2_DIR/lib/cmake/realsense2/realsense2-config.cmake")
  fi
  candidates+=("/usr/lib/x86_64-linux-gnu/cmake/realsense2/realsense2Config.cmake")
  candidates+=("/usr/lib/x86_64-linux-gnu/cmake/realsense2/realsense2-config.cmake")
  candidates+=("/usr/local/lib/x86_64-linux-gnu/cmake/realsense2/realsense2Config.cmake")
  candidates+=("/usr/local/lib/x86_64-linux-gnu/cmake/realsense2/realsense2-config.cmake")
  candidates+=("/usr/lib/cmake/realsense2/realsense2Config.cmake")
  candidates+=("/usr/lib/cmake/realsense2/realsense2-config.cmake")
  candidates+=("/usr/local/lib/cmake/realsense2/realsense2Config.cmake")
  candidates+=("/usr/local/lib/cmake/realsense2/realsense2-config.cmake")

  for cfg in "${candidates[@]}"; do
    if [[ -f "$cfg" ]]; then
      echo "$cfg"
      return 0
    fi
  done

  return 1
}

ensure_realsense_sdk() {
  local rs_version="${REALSENSE_VERSION:-2.57.5}"
  local rs_rel_root="${REALSENSE_ROOT_REL:-src/vision/realsense-ros/librealsense_sdk}"
  local rs_root="$WS_ROOT/$rs_rel_root"
  local rs_src="$rs_root"
  local rs_build="$rs_root/build"
  local rs_install="$rs_root/install"
  local rs_config="$rs_install/lib/cmake/realsense2/realsense2Config.cmake"
  local rs_colcon_build="${REALSENSE_COLCON_BUILD:-0}"
  local rs_archive="librealsense-${rs_version}.tar.gz"
  local rs_url="${REALSENSE_SDK_URL:-https://github.com/IntelRealSense/librealsense/archive/refs/tags/v${rs_version}.tar.gz}"
  local rs_url_fallback="https://codeload.github.com/IntelRealSense/librealsense/tar.gz/refs/tags/v${rs_version}"
  local rs_archive_path="$(dirname "$rs_root")/$rs_archive"
  local legacy_rs_root="$WS_ROOT/src/vision/realsense-ros/librealsense"

  if [[ "$legacy_rs_root" != "$rs_root" && -d "$legacy_rs_root" ]]; then
    # Avoid colcon duplicate package names if legacy librealsense exists
    touch "$legacy_rs_root/COLCON_IGNORE"
  fi
  if [[ -d "$rs_root" && "$rs_colcon_build" == "0" ]]; then
    # Avoid rebuilding librealsense2 on every colcon build
    touch "$rs_root/COLCON_IGNORE"
  fi
  if [[ -d "$rs_root" && "$rs_colcon_build" != "0" ]]; then
    rm -f "$rs_root/COLCON_IGNORE"
  fi

  if cfg="$(find_realsense_config)"; then
    export realsense2_DIR="$(dirname "$cfg")"
    print_color green "RealSense SDK ready: $(relpath_ws "$cfg")"
    return 0
  fi

  if [[ -f "$rs_config" ]]; then
    export realsense2_DIR="$rs_install/lib/cmake/realsense2"
    export CMAKE_PREFIX_PATH="$rs_install${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
    if [[ "$rs_colcon_build" == "0" ]]; then
      touch "$rs_root/COLCON_IGNORE"
    fi
    print_color green "RealSense SDK ready: $(relpath_ws "$rs_install")"
    return 0
  fi

  require_cmd curl tar cmake
  print_color yellow "RealSense SDK missing, preparing v${rs_version} ..."
  print_color yellow "Source: $rs_url"
  print_color yellow "Fallback: $rs_url_fallback"

  mkdir -p "$(dirname "$rs_root")"
  if [[ ! -f "$rs_src/CMakeLists.txt" ]]; then
    rm -f "$rs_archive_path"
    if download_with_fallback "$rs_archive_path" "$rs_url" "$rs_url_fallback"; then
      tar -xzf "$rs_archive_path" -C "$(dirname "$rs_root")"
      rm -f "$rs_archive_path"

      local extracted="$(dirname "$rs_root")/librealsense-${rs_version}"
      if [[ ! -d "$extracted" ]]; then
        extracted="$(find "$(dirname "$rs_root")" -maxdepth 1 -type d -name 'librealsense-*' | head -n 1 || true)"
      fi
      if [[ -z "${extracted:-}" || ! -d "$extracted" ]]; then
        print_color red "Failed to locate extracted librealsense source under: $(dirname "$rs_root")"
        return 1
      fi

      rm -rf "$rs_src"
      mv "$extracted" "$rs_src"
    else
      if command -v git >/dev/null 2>&1; then
        print_color yellow "Download failed, trying git clone ..."
        rm -rf "$rs_src"
        if ! git -c advice.detachedHead=false clone --branch "v${rs_version}" --depth 1 https://github.com/IntelRealSense/librealsense.git "$rs_src"; then
          print_color red "Failed to clone RealSense SDK from git"
          return 1
        fi
      else
        print_color red "Failed to download RealSense SDK from: $rs_url"
        return 1
      fi
    fi
  fi

  if [[ ! -f "$rs_src/CMakeLists.txt" ]]; then
    print_color red "Invalid librealsense source at: $rs_src"
    return 1
  fi

  rm -rf "$rs_build" "$rs_install"
  cmake -S "$rs_src" -B "$rs_build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$rs_install" \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_GRAPHICAL_EXAMPLES=OFF \
    -DBUILD_PYTHON_BINDINGS=OFF \
    -DBUILD_UNIT_TESTS=OFF \
    -DBUILD_WITH_CUDA=OFF \
    -DCHECK_FOR_UPDATES=OFF \
    -DIMPORT_DEPTH_CAM_FW=OFF

  local jobs=1
  if command -v nproc >/dev/null 2>&1; then
    jobs="$(nproc)"
  fi
  cmake --build "$rs_build" -j"$jobs"
  cmake --install "$rs_build"

  if [[ ! -f "$rs_config" ]]; then
    print_color red "RealSense SDK build completed but config not found: $rs_config"
    return 1
  fi

  export realsense2_DIR="$rs_install/lib/cmake/realsense2"
  export CMAKE_PREFIX_PATH="$rs_install${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
  if [[ "$rs_colcon_build" == "0" ]]; then
    touch "$rs_root/COLCON_IGNORE"
  fi
  print_color green "RealSense SDK prepared: $(relpath_ws "$rs_install")"
}

check_env() {
  ensure_onnxruntime
  ensure_realsense_sdk
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  check_env "$@"
fi
