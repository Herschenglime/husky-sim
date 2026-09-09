cmake_minimum_required(VERSION 3.8)
project(line_follow_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  worlds
  models
  scripts
  track_source
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
