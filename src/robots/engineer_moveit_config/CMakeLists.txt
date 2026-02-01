# ----------------------------------------------------------------------------
#  基础配置
# ----------------------------------------------------------------------------
# 指定 CMake 最低版本
cmake_minimum_required(VERSION 3.22)

# 声明工程名
project(engineer_moveit_config)

# ----------------------------------------------------------------------------
#  查找依赖
# ----------------------------------------------------------------------------

# 引入 ament 构建支持
find_package(ament_cmake REQUIRED)

# ----------------------------------------------------------------------------
#  安装与导出
# ----------------------------------------------------------------------------

# 生成包元数据
ament_package()

# 安装启动配置，跳过助手生成的模板
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

# 安装 MoveIt 配置与助手记录
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
