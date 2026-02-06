cmake_minimum_required(VERSION 3.13)
project(detect_node)

# =========================
# Compile options
# =========================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")

# =========================
# Dependencies
# =========================
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ---------------------------------------------------------
# PCL (关键修复：加 registration)
# 你用到了：
# - ICPWithNormals / TransformationEstimationPointToPlaneLLS  -> registration
# - NormalEstimationOMP / features
# - EuclideanClusterExtraction / segmentation
# ---------------------------------------------------------
find_package(PCL REQUIRED COMPONENTS
  common
  io
  filters
  segmentation
  search
  features
  kdtree
  registration
)

# =========================
# YOLOs-CPP paths (relative to this package)
# =========================
set(YOLOS_CPP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../YOLOs-CPP)
set(YOLOS_CPP_BUILD_DIR ${YOLOS_CPP_DIR}/build)
set(ONNXRUNTIME_DIR ${YOLOS_CPP_DIR}/onnxruntime-linux-x64-1.20.1)

# =========================
# Library
# =========================
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/detect_node.cpp
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(${PROJECT_NAME} PRIVATE
    -Wno-maybe-uninitialized
    -Wno-array-bounds
  )
endif()

# =========================
# Include directories
# =========================
target_include_directories(${PROJECT_NAME}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${YOLOS_CPP_DIR}/include
    ${ONNXRUNTIME_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

# =========================
# PCL compile definitions (有些发行版需要)
# =========================
# 例如 -DPCL_NO_PRECOMPILE 等会在这里给出
target_compile_definitions(${PROJECT_NAME}
  PRIVATE
    ${PCL_DEFINITIONS}
)

# =========================
# ROS dependencies
# =========================
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
  geometry_msgs
  OpenCV
  message_filters
  ament_index_cpp
)

# =========================
# Link libraries
# =========================
target_link_libraries(${PROJECT_NAME}
  yaml-cpp
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  ${YOLOS_CPP_BUILD_DIR}/libyolos_cpp.a
  ${ONNXRUNTIME_DIR}/lib/libonnxruntime.so
)

# =========================
# RPATH (Linux runtime fix)
# =========================
set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_RPATH ${ONNXRUNTIME_DIR}/lib
  INSTALL_RPATH ${ONNXRUNTIME_DIR}/lib
)

# =========================
# Register component
# =========================
rclcpp_components_register_nodes(${PROJECT_NAME}
  PLUGIN "arm_controller::DetectNode"
)

# =========================
# Install targets
# =========================
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# =========================
# Install launch files
# =========================
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# =========================
# Install YOLO models
# =========================
install(
  DIRECTORY ${YOLOS_CPP_DIR}/models/
  DESTINATION share/${PROJECT_NAME}/models
  FILES_MATCHING
    PATTERN "yolo11s-seg.onnx"
    PATTERN "coco.names"
)

ament_package()
