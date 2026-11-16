cmake_minimum_required(VERSION 3.16)
project(ur5_sim NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config urdf DESTINATION share/${PROJECT_NAME})

ament_package()
