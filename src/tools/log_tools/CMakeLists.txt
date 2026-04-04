cmake_minimum_required(VERSION 3.8)
project(log_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

add_library(log_tools INTERFACE)

set(SPDLOG_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../third_party/spdlog/include")

target_include_directories(log_tools INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/log_tools>
  $<BUILD_INTERFACE:${SPDLOG_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>
  $<INSTALL_INTERFACE:include/log_tools>
)

target_compile_features(log_tools INTERFACE cxx_std_11)

target_compile_definitions(log_tools INTERFACE
  SPDLOG_HEADER_ONLY
  SPDLOG_FMT_EXTERNAL=0
)

install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY "${SPDLOG_INCLUDE_DIR}/spdlog"
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_targets(log_toolsTargets)

install(TARGETS log_tools
  EXPORT log_toolsTargets
)

ament_package()
