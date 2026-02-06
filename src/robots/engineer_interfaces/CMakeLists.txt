# ----------------------------------------------------------------------------
#  基础配置
# ----------------------------------------------------------------------------
# 指定 CMake 最低版本
cmake_minimum_required(VERSION 3.8)

# 声明接口包名，保持与 package.xml 一致
project(engineer_interfaces)

# ----------------------------------------------------------------------------
#  编译选项
# ----------------------------------------------------------------------------
# 开启基础告警，提升接口定义质量
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ----------------------------------------------------------------------------
#  查找依赖
# ----------------------------------------------------------------------------

# 引入 ament 与消息生成工具
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ----------------------------------------------------------------------------
#  生成接口
# ----------------------------------------------------------------------------

# 声明消息与动作接口，统一生成代码
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Intent.msg"
  "msg/Joint.msg"
  "msg/Joints.msg"
  "msg/ModeCommand.msg"
  "msg/Target.msg"
  "msg/GripperCommand.msg"
  "action/Move.action"
  DEPENDENCIES 
    builtin_interfaces
    geometry_msgs
    std_msgs
)

# ----------------------------------------------------------------------------
#  包导出
# ----------------------------------------------------------------------------

# 输出接口包元数据
ament_package()
