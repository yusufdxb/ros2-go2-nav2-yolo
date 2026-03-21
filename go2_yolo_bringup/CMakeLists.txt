cmake_minimum_required(VERSION 3.5)
project(go2_yolo_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config worlds rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/scan_relay.py scripts/demo_scenario.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
