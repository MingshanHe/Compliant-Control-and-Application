cmake_minimum_required(VERSION 3.5.1)
cmake_policy(SET CMP0048 NEW)
project(mir_dwb_critics)

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;-Wall;-Werror")

find_package(catkin REQUIRED COMPONENTS
  angles
  costmap_queue
  dwb_critics
  dwb_local_planner
  geometry_msgs
  nav_2d_msgs
  nav_2d_utils
  nav_core2
  nav_grid_iterators
  pluginlib
  roscpp
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS angles costmap_queue dwb_critics dwb_local_planner geometry_msgs nav_2d_msgs nav_2d_utils nav_core2 nav_grid_iterators pluginlib roscpp sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/path_angle.cpp
  src/path_progress.cpp
  src/path_dist_pruned.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(PROGRAMS
    nodes/print_dwb_scores.py
    nodes/plot_dwb_scores.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES default_critics.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
