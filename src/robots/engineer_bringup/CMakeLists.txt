# ----------------------------------------------------------------------------
#  基础配置
# ----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.8)
project(engineer_bringup)

# ----------------------------------------------------------------------------
#  编译选项
# ----------------------------------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ----------------------------------------------------------------------------
#  查找依赖
# ----------------------------------------------------------------------------

# 引入 ament 构建支持
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# ----------------------------------------------------------------------------
#  安装与导出
# ----------------------------------------------------------------------------

# 安装运行所需的配置与启动文件
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# 输出包元数据
ament_package()
