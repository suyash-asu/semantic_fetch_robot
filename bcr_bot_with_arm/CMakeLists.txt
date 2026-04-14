cmake_minimum_required(VERSION 3.8)
project(bcr_bot_with_arm)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
