# ----------------------------------------------------------------------------
#  基础配置
# ----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.8)
project(params_utils)

# ----------------------------------------------------------------------------
#  编译选项
# ----------------------------------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ----------------------------------------------------------------------------
#  查找依赖
# ----------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)

# ----------------------------------------------------------------------------
#  安装与导出
# ----------------------------------------------------------------------------
install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp rcl_interfaces)

ament_package()
