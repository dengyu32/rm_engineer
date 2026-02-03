# ----------------------------------------------------------------------------
#  基础配置
# ----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.8)
project(arm_servo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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

# 引入构建与控制相关依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(engineer_interfaces REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(error_code_utils REQUIRED)

# ----------------------------------------------------------------------------
#  构建库与可执行文件
# ----------------------------------------------------------------------------

# 将伺服节点编译为共享库
add_library(${PROJECT_NAME} SHARED
  src/arm_servo_node.cpp
)

# 配置头文件可见性
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 绑定运行时依赖
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  trajectory_msgs
  engineer_interfaces
  error_code_utils
)

# 注册组件并生成可执行入口
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "arm_servo::ArmServoNode"
  EXECUTABLE arm_servo_node
)

# 额外的键盘控制可执行体
add_executable(servo_keyboard_input src/servo_keyboard_input.cpp)
ament_target_dependencies(servo_keyboard_input
  rclcpp
  geometry_msgs
  control_msgs
)

# ----------------------------------------------------------------------------
#  安装与导出
# ----------------------------------------------------------------------------

# 安装库与节点可执行体
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装头文件供下游使用
install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS servo_keyboard_input
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装运行配置
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

# ----------------------------------------------------------------------------
#  测试
# ----------------------------------------------------------------------------

# 配置 lint 测试依赖
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 生成包元数据
ament_package()
