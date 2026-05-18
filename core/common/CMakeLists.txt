cmake_minimum_required(VERSION 3.8)
project(common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

file(GLOB util_src_files src/util/*.cpp)
file(GLOB math_src_files_cpp src/math/*.cpp)
file(GLOB geometry_src_files src/geometry/*.cpp)
file(GLOB geometry_curve_src_files src/geometry/curve/*.cpp)

add_library(${PROJECT_NAME} SHARED
  ${util_src_files}
  ${math_src_files_cpp}
  ${geometry_src_files}
  ${geometry_curve_src_files}
)

target_link_libraries(${PROJECT_NAME}
  glog::glog
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
  $<INSTALL_INTERFACE:include/common>
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  nav2_costmap_2d
  tf2
  tf2_ros
  nav_msgs
  std_msgs
  visualization_msgs
  geometry_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY src/
  DESTINATION include/common
  FILES_MATCHING PATTERN "*.h"
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  glog
  rclcpp
  nav2_costmap_2d
  tf2
  tf2_ros
  nav_msgs
  std_msgs
  visualization_msgs
  geometry_msgs
  Eigen3
)
ament_export_include_directories(include)
ament_export_include_directories(include/common)
ament_export_libraries(${PROJECT_NAME})

ament_package()
