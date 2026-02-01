# ----------------------------------------------------------------------------
#  基础配置
# ----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.8)
project(top_hfsm)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 共享接口头路径
set(MY_ROBOT_INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/../../robots/engineer/engineer_interface")

# ----------------------------------------------------------------------------
#  编译选项
# ----------------------------------------------------------------------------
# 启用基础告警，忽略部分参数未使用提示
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# ----------------------------------------------------------------------------
#  查找依赖
# ----------------------------------------------------------------------------

# 引入节点所需的通信与动作依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(engineer_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(param_utils REQUIRED)
find_package(error_code_utils REQUIRED)

# ----------------------------------------------------------------------------
#  构建库和可执行文件
# ----------------------------------------------------------------------------

# 将 HFSM 节点实现编译为共享库
add_library(${PROJECT_NAME} SHARED 
  src/executors/arm_solve_client.cpp 
  src/executors/vision_detect_client.cpp 
  src/executors/gripper_control_node.cpp
  src/executors/moveit_servo_client.cpp          
  src/top_hfsm_node.cpp
)

# 配置头文件可见性
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${MY_ROBOT_INTERFACE}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  # 本项目 include
  $<INSTALL_INTERFACE:include>
)

# 绑定运行时依赖
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  std_msgs
  engineer_interfaces
  rclcpp_action
  tf2_geometry_msgs
  std_srvs
  control_msgs
  param_utils
  error_code_utils
)

# 注册组件并生成可执行入口
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN top_hfsm::TopHFSMNode
  EXECUTABLE top_hfsm_node
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

# 安装头文件供下游使用
install(DIRECTORY include/ DESTINATION include/)

# 安装运行配置
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

# 生成包元数据
ament_package()
