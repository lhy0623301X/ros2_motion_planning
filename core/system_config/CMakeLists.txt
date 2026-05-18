cmake_minimum_required(VERSION 3.8)
project(system_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(common REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(glog REQUIRED)
find_package(Protobuf 3 REQUIRED)

add_library(${PROJECT_NAME}
  src/proto_util.cpp
  src/system_config.cpp
)

# Protobuf generation
set(PROTO_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/protos)
file(MAKE_DIRECTORY ${PROTO_OUTPUT_DIR})

# common related protos
file(GLOB_RECURSE COMMON_PROTOS "${CMAKE_CURRENT_SOURCE_DIR}/system_config/common_protos/*.proto")
protobuf_generate(
  TARGET ${PROJECT_NAME}
  LANGUAGE cpp
  OUT_VAR COMMON_PROTO_SRCS
  PROTOS ${COMMON_PROTOS}
)

# path planner related protos
file(GLOB_RECURSE PATH_PLANNER_PROTOS "${CMAKE_CURRENT_SOURCE_DIR}/system_config/path_planner_protos/*.proto")
protobuf_generate(
  TARGET ${PROJECT_NAME}
  LANGUAGE cpp
  OUT_VAR PATH_PLANNER_PROTO_SRCS
  PROTOS ${PATH_PLANNER_PROTOS}
)

# controller related protos
file(GLOB CONTROLLER_PROTOS "${CMAKE_CURRENT_SOURCE_DIR}/system_config/controller_protos/*.proto")
protobuf_generate(
  TARGET ${PROJECT_NAME}
  LANGUAGE cpp
  OUT_VAR CONTROLLER_PROTO_SRCS
  PROTOS ${CONTROLLER_PROTOS}
)

# system configure related protos
file(GLOB SYSTEM_CONFIG_PROTOS "${CMAKE_CURRENT_SOURCE_DIR}/system_config/*.proto")
protobuf_generate(
  TARGET ${PROJECT_NAME}
  LANGUAGE cpp
  OUT_VAR SYSTEM_CONFIG_PROTO_SRCS
  PROTOS ${SYSTEM_CONFIG_PROTOS}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  glog::glog
  ${Protobuf_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  common
  ament_index_cpp
)

# Install
install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY system_config/
  DESTINATION share/${PROJECT_NAME}/system_config
)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(common Protobuf ament_index_cpp glog)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
