cmake_minimum_required(VERSION 3.8)
project(hub_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake QUIET)

add_library(hub_utils INTERFACE)

target_include_directories(hub_utils INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(hub_utils INTERFACE cxx_std_17)

if(TARGET error_code_utils)
  target_link_libraries(hub_utils INTERFACE error_code_utils)
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../error_code_utils/include)
  target_include_directories(hub_utils INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../error_code_utils/include>
  )
endif()

option(HUB_UTILS_BUILD_TESTS "Build hub_utils tests" OFF)
if(BUILD_TESTING OR HUB_UTILS_BUILD_TESTS)
  include(CTest)
  add_executable(test_aggregator tests/test_aggregator.cpp)
  target_link_libraries(test_aggregator hub_utils)
  if(TARGET error_code_utils)
    target_link_libraries(test_aggregator error_code_utils)
  endif()
  add_test(NAME hub_utils_aggregator COMMAND test_aggregator)
endif()

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS hub_utils
  EXPORT hub_utilsTargets
)

if(ament_cmake_FOUND)
  ament_export_targets(hub_utilsTargets HAS_LIBRARY_TARGET)
  ament_export_include_directories(include)
  ament_package()
endif()
