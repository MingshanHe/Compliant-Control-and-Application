cmake_minimum_required(VERSION 3.16)
project(robot_sim_bringup NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch models rviz urdf worlds DESTINATION share/${PROJECT_NAME})

ament_package()
