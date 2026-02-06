cmake_minimum_required(VERSION 3.8)
project(error_code_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake QUIET)

add_library(error_code_utils INTERFACE)

target_include_directories(error_code_utils INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(error_code_utils INTERFACE cxx_std_17)

option(ERROR_CODE_UTILS_BUILD_TESTS "Build error_code_utils tests" OFF)
if(BUILD_TESTING OR ERROR_CODE_UTILS_BUILD_TESTS)
  include(CTest)
  add_executable(test_counter tests/test_counter.cpp)
  target_link_libraries(test_counter error_code_utils)
  add_test(NAME error_code_utils_counter COMMAND test_counter)

  add_executable(test_latch_ack tests/test_latch_ack.cpp)
  target_link_libraries(test_latch_ack error_code_utils)
  add_test(NAME error_code_utils_latch_ack COMMAND test_latch_ack)
endif()

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS error_code_utils
  EXPORT error_code_utilsTargets
)

if(ament_cmake_FOUND)
  ament_export_include_directories(include)
  ament_export_targets(error_code_utilsTargets)
  ament_package()
endif()
