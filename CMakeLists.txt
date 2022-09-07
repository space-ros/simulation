cmake_minimum_required(VERSION 3.8)
project(simulation)
find_package(ament_cmake REQUIRED)

install(DIRECTORY
  models
  DESTINATION share/${PROJECT_NAME}/
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
