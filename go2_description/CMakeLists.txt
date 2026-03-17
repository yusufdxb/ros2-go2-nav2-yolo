cmake_minimum_required(VERSION 3.5)
project(go2_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY xacro meshes config dae
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS xacro/xacro_no_decl.sh
  DESTINATION share/${PROJECT_NAME}/xacro
)

ament_package()
