cmake_minimum_required(VERSION 3.8)
project(sim_env)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  worlds
  maps
  meshes
  models
  urdf
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/load_world_models.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
