cmake_minimum_required(VERSION 3.8)
project(ur7e_robotiq_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  urdf
  config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/gripper.sh
  scripts/moveit_test.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
