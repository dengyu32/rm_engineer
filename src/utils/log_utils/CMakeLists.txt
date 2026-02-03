cmake_minimum_required(VERSION 3.8)
project(log_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

add_library(log_utils INTERFACE)

target_include_directories(log_utils INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/log_utils>
  $<INSTALL_INTERFACE:include>
  $<INSTALL_INTERFACE:include/log_utils>
)

target_compile_features(log_utils INTERFACE cxx_std_11)

target_compile_definitions(log_utils INTERFACE
  SPDLOG_HEADER_ONLY
  SPDLOG_FMT_EXTERNAL=0
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_targets(log_utilsTargets)

install(TARGETS log_utils
  EXPORT log_utilsTargets
)

ament_package()
