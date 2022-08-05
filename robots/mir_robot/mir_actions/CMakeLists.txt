cmake_minimum_required(VERSION 3.5.1)
cmake_policy(SET CMP0048 NEW)
project(mir_actions)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  message_generation
  nav_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate actions in the 'action' folder
add_action_files(
  FILES
    MirMoveBase.action
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    geometry_msgs
    nav_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    actionlib
    geometry_msgs
    message_runtime
    nav_msgs
)
