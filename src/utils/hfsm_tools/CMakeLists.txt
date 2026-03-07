cmake_minimum_required(VERSION 3.5)
project(hfsm_tools)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(engineer_interfaces REQUIRED)

add_executable(hfsm_intent_runner src/hfsm_intent_runner.cpp)
ament_target_dependencies(hfsm_intent_runner rclcpp std_msgs engineer_interfaces)
target_compile_features(hfsm_intent_runner PUBLIC cxx_std_17)

install(TARGETS hfsm_intent_runner DESTINATION lib/${PROJECT_NAME})
install(FILES resource/${PROJECT_NAME} DESTINATION share/ament_index/resource_index/packages)
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

ament_package()
