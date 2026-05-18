cmake_minimum_required(VERSION 3.8)
project(path_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(common REQUIRED)
find_package(glog REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/path_planner.cpp
  src/path_planner_node.cpp
  src/graph_planner/astar_planner.cpp
  src/graph_planner/dijkstra_planner.cpp
  src/graph_planner/dstar_planner.cpp
  src/graph_planner/dstar_lite_planner.cpp
  src/graph_planner/gbfs_planner.cpp
  src/graph_planner/jps_planner.cpp
  src/graph_planner/lpa_star_planner.cpp
  src/graph_planner/hybrid_astar_planner/hybrid_astar_planner.cpp
  src/graph_planner/hybrid_astar_planner/motion_table.cpp
  src/graph_planner/hybrid_astar_planner/node_hybrid.cpp
  src/utils/path_planner_factory.cpp
  src/utils/path_replanning_utils.cpp
)

target_link_libraries(${PROJECT_NAME}
  glog::glog
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(${PROJECT_NAME}
  common
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_ros
)

pluginlib_export_plugin_description_file(nav2_core path_planner_plugin.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  FILES path_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY src/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  common
  glog
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_ros
)

ament_package()
