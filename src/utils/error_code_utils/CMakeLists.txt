cmake_minimum_required(VERSION 3.8)
project(error_code_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

add_library(error_code_utils INTERFACE)

target_include_directories(error_code_utils INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(error_code_utils INTERFACE cxx_std_11)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_targets(error_code_utilsTargets)

install(TARGETS error_code_utils
  EXPORT error_code_utilsTargets
)

ament_package()
