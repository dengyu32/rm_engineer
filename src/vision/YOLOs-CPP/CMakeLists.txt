# ============================================================================
# YOLOs-CPP - High-Performance C++ Inference for YOLO Models
# ============================================================================
cmake_minimum_required(VERSION 3.16)
project(YOLOs-CPP VERSION 2.0.0 LANGUAGES CXX)

# ============================================================================
# C++ Standard
# ============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ============================================================================
# Build Options
# ============================================================================
option(BUILD_EXAMPLES "Build example applications" OFF)
option(ENABLE_WARNINGS "Enable compiler warnings" ON)

# ============================================================================
# ONNX Runtime
# ============================================================================
if(NOT DEFINED ONNXRUNTIME_DIR)
    if(WIN32)
        set(ONNXRUNTIME_SEARCH_PATHS
            "${CMAKE_SOURCE_DIR}/onnxruntime-win-x64-1.20.1"
            "${CMAKE_SOURCE_DIR}/onnxruntime-win-x64-gpu-1.20.1"
            "${CMAKE_SOURCE_DIR}/onnxruntime"
            "$ENV{ONNXRUNTIME_DIR}"
        )
    elseif(APPLE)
        set(ONNXRUNTIME_SEARCH_PATHS
            "${CMAKE_SOURCE_DIR}/onnxruntime-osx-arm64-1.20.1"
            "${CMAKE_SOURCE_DIR}/onnxruntime-osx-x86_64-1.20.1"
            "${CMAKE_SOURCE_DIR}/onnxruntime"
            "/opt/onnxruntime"
            "$ENV{ONNXRUNTIME_DIR}"
        )
    else()
        set(ONNXRUNTIME_SEARCH_PATHS
            "${CMAKE_SOURCE_DIR}/onnxruntime-linux-x64-1.20.1"
            "${CMAKE_SOURCE_DIR}/onnxruntime-linux-x64-gpu-1.20.1"
            "${CMAKE_SOURCE_DIR}/onnxruntime"
            "/opt/onnxruntime"
            "$ENV{ONNXRUNTIME_DIR}"
        )
    endif()

    foreach(path ${ONNXRUNTIME_SEARCH_PATHS})
        if(EXISTS "${path}/include/onnxruntime_cxx_api.h")
            set(ONNXRUNTIME_DIR "${path}")
            break()
        endif()
    endforeach()
endif()

if(ONNXRUNTIME_DIR AND NOT IS_ABSOLUTE "${ONNXRUNTIME_DIR}")
    get_filename_component(ONNXRUNTIME_DIR "${ONNXRUNTIME_DIR}" ABSOLUTE BASE_DIR "${CMAKE_CURRENT_BINARY_DIR}")
endif()

if(NOT ONNXRUNTIME_DIR OR NOT EXISTS "${ONNXRUNTIME_DIR}/include")
    message(FATAL_ERROR 
        "ONNX Runtime not found!\n"
        "cmake .. -DONNXRUNTIME_DIR=/path/to/onnxruntime"
    )
endif()

message(STATUS "ONNX Runtime: ${ONNXRUNTIME_DIR}")

# ============================================================================
# OpenCV
# ============================================================================
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV: ${OpenCV_VERSION}")

# ============================================================================
# Threads
# ============================================================================
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# ============================================================================
# Compiler Flags
# ============================================================================
if(ENABLE_WARNINGS)
    if(MSVC)
        add_compile_options(/W4)
    else()
        add_compile_options(-Wall -Wextra -Wpedantic)
    endif()
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    if(NOT MSVC)
        add_compile_options(-O3)
    endif()
endif()

# ============================================================================
# ONNX Runtime Library
# ============================================================================
if(UNIX AND NOT APPLE)
    set(ONNXRUNTIME_LIB "${ONNXRUNTIME_DIR}/lib/libonnxruntime.so")
elseif(APPLE)
    set(ONNXRUNTIME_LIB "${ONNXRUNTIME_DIR}/lib/libonnxruntime.dylib")
elseif(WIN32)
    set(ONNXRUNTIME_LIB "${ONNXRUNTIME_DIR}/lib/onnxruntime.lib")
endif()

# ============================================================================
# ==================  核心修改部分  ==================
# Create YOLOs-CPP as a STATIC library
# ============================================================================
add_library(yolos_cpp STATIC
    # 强制生成库（header-only trick）
    src/image_inference.cpp
)

target_include_directories(yolos_cpp PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${ONNXRUNTIME_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(yolos_cpp PUBLIC
    ${OpenCV_LIBS}
    ${ONNXRUNTIME_LIB}
    Threads::Threads
)

# Linux RPATH
if(UNIX AND NOT APPLE)
    set_target_properties(yolos_cpp PROPERTIES
        BUILD_RPATH "${ONNXRUNTIME_DIR}/lib"
        INSTALL_RPATH "${ONNXRUNTIME_DIR}/lib"
    )
endif()

# ============================================================================
# Helper Function (for demo executables)
# ============================================================================
function(add_yolo_executable name source)
    add_executable(${name} ${source})
    target_link_libraries(${name} PRIVATE yolos_cpp)
endfunction()

# ============================================================================
# Demo Executables
# ============================================================================
add_yolo_executable(image_inference src/image_inference.cpp)
add_yolo_executable(video_inference src/video_inference.cpp)
add_yolo_executable(camera_inference src/camera_inference.cpp)
add_yolo_executable(class_image_inference src/class_image_inference.cpp)
add_yolo_executable(batch_image_inference src/batch_image_inference.cpp)

# ============================================================================
# Install
# ============================================================================
install(TARGETS
    yolos_cpp
    image_inference
    video_inference
    camera_inference
    class_image_inference
    batch_image_inference
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# ============================================================================
# Examples
# ============================================================================
if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# ============================================================================
# Summary
# ============================================================================
message(STATUS "")
message(STATUS "=== YOLOs-CPP Configuration ===")
message(STATUS "  Version:       ${PROJECT_VERSION}")
message(STATUS "  Build Type:    ${CMAKE_BUILD_TYPE}")
message(STATUS "  OpenCV:        ${OpenCV_VERSION}")
message(STATUS "  ONNX Runtime:  ${ONNXRUNTIME_DIR}")
message(STATUS "")
