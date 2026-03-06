cmake_minimum_required(VERSION 3.8)
project(teleop_input_adapter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(engineer_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(teleop_input_adapter src/teleop_input_adapter.cpp)
target_include_directories(teleop_input_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(teleop_input_adapter
  control_msgs
  engineer_interfaces
  rclcpp
)

install(TARGETS teleop_input_adapter
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
