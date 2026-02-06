cmake_minimum_required(VERSION 3.8)
project(service_hub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# 如果你用到 std_srvs / std_msgs / 你的自定义接口，也在这里 find_package
# find_package(std_srvs REQUIRED)
# find_package(engineer_interfaces REQUIRED)

add_library(service_hub
  src/service_hub.cpp
)

target_include_directories(service_hub
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(service_hub PUBLIC cxx_std_17)

ament_target_dependencies(service_hub
  PUBLIC
    rclcpp
    # std_srvs
    # engineer_interfaces
)

install(TARGETS service_hub
  EXPORT service_hubTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(service_hubTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)
# ament_export_dependencies(std_srvs engineer_interfaces)

ament_package()