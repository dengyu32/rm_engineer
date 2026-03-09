cmake_minimum_required(VERSION 3.16)
project(object_load)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(object_load src/object_load_node.cpp)
ament_target_dependencies(object_load
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  shape_msgs
  geometric_shapes
)
target_link_libraries(object_load Eigen3::Eigen)

install(TARGETS object_load
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
