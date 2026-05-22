cmake_minimum_required(VERSION 3.8)
project(controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(common REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

find_library(G2O_CORE_LIB g2o_core PATHS /opt/ros/humble/lib/x86_64-linux-gnu REQUIRED)
find_library(G2O_STUFF_LIB g2o_stuff PATHS /opt/ros/humble/lib/x86_64-linux-gnu REQUIRED)
find_library(
  G2O_SOLVER_CSPARSE_LIB g2o_solver_csparse
  PATHS /opt/ros/humble/lib/x86_64-linux-gnu REQUIRED)
find_library(
  G2O_CSPARSE_EXTENSION_LIB g2o_csparse_extension
  PATHS /opt/ros/humble/lib/x86_64-linux-gnu REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/controller_algorithm.cpp
  src/controller_factory.cpp
  src/controller_node.cpp
  src/dwa_controller/dwa_controller.cpp
  src/dwa_controller/dwa_critic.cpp
  src/dwa_controller/dwa_motion.cpp
  src/dwa_controller/dwa_visualizer.cpp
  src/lqr_controller/lqr_controller.cpp
  src/mpc_controller/mpc_controller.cpp
  src/pid_controller/pid_controller.cpp
  src/rpp_controller/rpp_controller.cpp
  src/teb_controller/teb_controller.cpp
  src/teb_controller/teb_optimizer.cpp
  src/teb_controller/teb_visualizer.cpp
  src/utils/curvature_speed_limiter.cpp
  src/utils/heading_aligner.cpp
  src/utils/lookahead_point_publisher.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    ${EIGEN3_INCLUDE_DIR}
    /opt/ros/humble/include
    /usr/include/suitesparse
)

target_link_libraries(${PROJECT_NAME}
  ${G2O_CORE_LIB}
  ${G2O_STUFF_LIB}
  ${G2O_SOLVER_CSPARSE_LIB}
  ${G2O_CSPARSE_EXTENSION_LIB}
  OsqpEigen::OsqpEigen
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_RPATH "/usr/local/lib"
  INSTALL_RPATH "/usr/local/lib"
  INSTALL_RPATH_USE_LINK_PATH TRUE
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
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

pluginlib_export_plugin_description_file(nav2_core controller_plugin.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  FILES
    src/controller_algorithm.h
    src/controller_factory.h
    src/controller_node.h
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY
    src/dwa_controller
    src/lqr_controller
    src/mpc_controller
    src/pid_controller
    src/rpp_controller
    src/teb_controller
    src/utils
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

install(
  FILES controller_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  common
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)
ament_export_libraries(${PROJECT_NAME})

ament_package()
