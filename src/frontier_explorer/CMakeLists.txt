cmake_minimum_required(VERSION 3.8)
project(frontier_explorer)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/wait_for_sim.py
    scripts/wait_for_costmap.py
    scripts/initial_spin.py
    scripts/resume_explore.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
