# ----------------------------------------------------------------------------
#  基础配置
# ----------------------------------------------------------------------------
# 指定 CMake 最低版本
cmake_minimum_required(VERSION 3.8)

# 声明工程名，与 package.xml 对齐
project(engineer_description)

# ----------------------------------------------------------------------------
#  编译选项
# ----------------------------------------------------------------------------
# 为 GCC/Clang 开启基础告警
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 统一使用 C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ----------------------------------------------------------------------------
#  查找依赖
# ----------------------------------------------------------------------------

# 加载 ament 构建支持
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# ----------------------------------------------------------------------------
#  安装与导出
# ----------------------------------------------------------------------------

# 安装机器人描述资源
install(
  DIRECTORY urdf meshes 
  DESTINATION share/${PROJECT_NAME}
)

# 生成包元数据
ament_package()
