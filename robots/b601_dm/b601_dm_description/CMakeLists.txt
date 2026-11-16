cmake_minimum_required(VERSION 3.16)
project(b601_dm_description NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch meshes urdf DESTINATION share/${PROJECT_NAME})

ament_package()
