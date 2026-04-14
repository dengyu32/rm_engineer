cmake_minimum_required(VERSION 3.16)
project(coal VERSION 3.0.2 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

add_library(coal SHARED
  src/BV/AABB.cpp
  src/BV/OBB.cpp
  src/BV/OBBRSS.cpp
  src/BV/RSS.cpp
  src/BV/kDOP.cpp
  src/BV/kIOS.cpp
  src/BVH/BVH_model.cpp
  src/BVH/BVH_utility.cpp
  src/BVH/BV_fitter.cpp
  src/BVH/BV_splitter.cpp
  src/collision.cpp
  src/collision_data.cpp
  src/collision_func_matrix.cpp
  src/collision_node.cpp
  src/collision_object.cpp
  src/collision_utility.cpp
  src/distance.cpp
  src/distance/box_halfspace.cpp
  src/distance/box_plane.cpp
  src/distance/box_sphere.cpp
  src/distance/capsule_capsule.cpp
  src/distance/capsule_halfspace.cpp
  src/distance/capsule_plane.cpp
  src/distance/cone_halfspace.cpp
  src/distance/cone_plane.cpp
  src/distance/convex_halfspace.cpp
  src/distance/convex_plane.cpp
  src/distance/cylinder_halfspace.cpp
  src/distance/cylinder_plane.cpp
  src/distance/ellipsoid_halfspace.cpp
  src/distance/ellipsoid_plane.cpp
  src/distance/halfspace_halfspace.cpp
  src/distance/halfspace_plane.cpp
  src/distance/plane_plane.cpp
  src/distance/sphere_capsule.cpp
  src/distance/sphere_cylinder.cpp
  src/distance/sphere_halfspace.cpp
  src/distance/sphere_plane.cpp
  src/distance/sphere_sphere.cpp
  src/distance/triangle_halfspace.cpp
  src/distance/triangle_plane.cpp
  src/distance/triangle_sphere.cpp
  src/distance/triangle_triangle.cpp
  src/distance_func_matrix.cpp
  src/hfield.cpp
  src/intersect.cpp
  src/math/transform.cpp
  src/narrowphase/gjk.cpp
  src/narrowphase/minkowski_difference.cpp
  src/narrowphase/support_functions.cpp
  src/shape/geometric_shapes.cpp
  src/shape/geometric_shapes_utility.cpp
  src/traversal/traversal_recurse.cpp
)
add_library(coal::coal ALIAS coal)

target_include_directories(coal
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/coal/third_party>
    $<INSTALL_INTERFACE:include>
    $<INSTALL_INTERFACE:include/coal/third_party>
    ${Boost_INCLUDE_DIRS}
)

target_link_libraries(coal PUBLIC Eigen3::Eigen)

target_compile_definitions(coal
  PRIVATE COAL_BUILDING_LIBRARY
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(coal PRIVATE -Wno-deprecated-declarations)
endif()

include(CMakePackageConfigHelpers)

install(TARGETS coal
  EXPORT coalTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(EXPORT coalTargets
  FILE coalTargets.cmake
  NAMESPACE coal::
  DESTINATION lib/cmake/coal
)

configure_package_config_file(
  cmake/coalConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/coalConfig.cmake
  INSTALL_DESTINATION lib/cmake/coal
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/coalConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/coalConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/coalConfigVersion.cmake
  DESTINATION lib/cmake/coal
)
