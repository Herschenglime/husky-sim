cmake_minimum_required(VERSION 3.8)
project(nav_worlds)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch maps worlds
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/send_goal.py
  scripts/bringup.sh
  scripts/scan_self_filter.py
  scripts/readiness_gate.py
  scripts/generate_waypoints.py
  scripts/log_state.py
  scripts/run_sweep.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
