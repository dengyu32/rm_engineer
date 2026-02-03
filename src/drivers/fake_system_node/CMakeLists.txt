# ----------------------------------------------------------------------------
#  基础配置
# ----------------------------------------------------------------------------
# 指定 CMake 最低版本
cmake_minimum_required(VERSION 3.8)

# 声明工程名
project(fake_system)

# 统一使用 C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 共享接口头路径
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

# 引入构建与消息依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(engineer_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(error_code_utils REQUIRED)
find_package(log_utils REQUIRED)

# ----------------------------------------------------------------------------
#  构建节点库
# ----------------------------------------------------------------------------

# 将节点实现编译为共享库
add_library(${PROJECT_NAME} SHARED src/fake_system_node.cpp)

# 配置头文件可见性
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${MY_ROBOT_INTERFACE}>
  $<INSTALL_INTERFACE:include>
)

# 绑定运行时依赖
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  engineer_interfaces
  sensor_msgs
  error_code_utils
  log_utils
)

# 注册组件并生成可执行入口
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "fake_system::FakeSystemNode"
  EXECUTABLE fake_system_node
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

install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}/
)

# 安装运行配置
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# 生成包元数据
ament_package()
