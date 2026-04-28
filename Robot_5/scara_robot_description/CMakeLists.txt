cmake_minimum_required(VERSION 3.8)
project(scara_robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY meshes urdf launch STL config scripts trayectorias worlds
  DESTINATION share/${PROJECT_NAME}
)


install(PROGRAMS
  scripts/scara_full_trajectory.py
  scripts/scara_trayectoria.py
  scripts/trajectory_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
