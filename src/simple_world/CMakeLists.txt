cmake_minimum_required(VERSION 3.5)
project(simple_world)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
