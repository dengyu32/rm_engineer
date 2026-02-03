# ----------------------------------------------------------------------------
#  基础配置
# ----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.8)
project(usb_cdc)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 共享接口头路径，供目标复用
set(MY_ROBOT_INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/../../robots/engineer/engineer_interface")

# ----------------------------------------------------------------------------
#  编译选项
# ----------------------------------------------------------------------------
# 启用基础告警，提前暴露问题
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ----------------------------------------------------------------------------
#  查找依赖
# ----------------------------------------------------------------------------

# 引入节点运行所需的 ROS 组件与消息包
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(engineer_interfaces REQUIRED)
find_package(error_code_utils REQUIRED)

# 通过 pkg-config 查询系统库
find_package(PkgConfig REQUIRED)

# 获取 libusb-1.0，用于 USB 通讯
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

# ----------------------------------------------------------------------------
#  构建节点库
# ----------------------------------------------------------------------------

# 将节点实现编译为共享库
add_library(${PROJECT_NAME} SHARED 
  src/usb_cdc_node.cpp
  src/usb_cdc_driver.cpp
)

# 暴露工程与外部依赖头文件目录
target_include_directories(${PROJECT_NAME} PUBLIC
  ${LIBUSB_INCLUDE_DIRS}  # libusb 头文件
  $<BUILD_INTERFACE:${MY_ROBOT_INTERFACE}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  # 本项目 include
)

# 注入 libusb 库目录
target_link_directories(${PROJECT_NAME} PUBLIC
  ${LIBUSB_LIBRARY_DIRS}
)

# 关联 USB 底层库
target_link_libraries(${PROJECT_NAME}
  ${LIBUSB_LIBRARIES}
)

# 绑定 ROS 运行时依赖
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs
  tf2_ros
  sensor_msgs
  engineer_interfaces
  error_code_utils
)

# 注册组件，生成可执行入口
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN usb_cdc::UsbCdcNode
  EXECUTABLE usb_cdc_node
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

# 分发启动文件，支持 ros2 launch
install(DIRECTORY  launch/
    DESTINATION share/${PROJECT_NAME}/
)


# 生成包元数据
ament_package()
