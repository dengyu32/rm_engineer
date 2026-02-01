# ----------------------------------------------------------------------------
#  基础配置
# ----------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.8)
project(arm_solve)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set(MY_ROBOT_INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/../../robots/engineer/engineer_interfaces")
# 共享接口头路径，供目标复用
set(MY_ROBOT_INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/../../robots/engineer/engineer_interfaces")

# ----------------------------------------------------------------------------
#  编译选项
# ----------------------------------------------------------------------------
# 启用基础告警
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ----------------------------------------------------------------------------
#  查找依赖
# ----------------------------------------------------------------------------

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(engineer_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_core REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(param_utils REQUIRED)
find_package(solve_core REQUIRED)
find_package(error_code_utils REQUIRED)

# ----------------------------------------------------------------------------
#  构建库和可执行文件
# ----------------------------------------------------------------------------

# 将节点实现编译为共享库
add_library(${PROJECT_NAME} SHARED src/arm_solve_server.cpp)

# 配置头文件可见性
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  # $<BUILD_INTERFACE:${MY_ROBOT_INTERFACE}>
  $<INSTALL_INTERFACE:include>
)

# 绑定运行时依赖
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_visual_tools
  engineer_interfaces
  geometry_msgs
  rclcpp_action
  moveit_core
  tf2
  tf2_eigen
  tf2_geometry_msgs
  param_utils
  solve_core
  error_code_utils
)

# 注册组件并生成可执行入口
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "arm_solve::ArmSolveServer"
  EXECUTABLE arm_solve_server
)

# ----------------------------------------------------------------------------
#  安装与导出
# ----------------------------------------------------------------------------

# 安装库与可执行体
install(TARGETS ${PROJECT_NAME} 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin 
)

# 安装运行配置
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# 生成包元数据
ament_package()
