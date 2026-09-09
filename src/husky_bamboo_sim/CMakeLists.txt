cmake_minimum_required(VERSION 3.8)
project(husky_bamboo_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds config maps
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/generate_bamboo_world.py
  scripts/sim_keyboard_teleop.py
  scripts/sim_debug_monitor.py
  scripts/sim_debug_monitor.sh
  scripts/ensure_controllers.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
