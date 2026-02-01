cmake_minimum_required(VERSION 3.8)
project(param_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_library(param_utils INTERFACE)
target_include_directories(param_utils INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(param_utils INTERFACE rclcpp)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_export_targets(param_utilsTargets)

install(TARGETS param_utils
  EXPORT param_utilsTargets
)

ament_package()
