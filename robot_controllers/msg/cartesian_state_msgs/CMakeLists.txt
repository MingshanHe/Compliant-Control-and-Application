cmake_minimum_required(VERSION 2.8.3)
project(cartesian_state_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  PoseTwist.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS geometry_msgs message_runtime)
