cmake_minimum_required(VERSION 2.8.3)

project(ur_e_gazebo)

find_package(catkin REQUIRED)
set(CMAKE_CXX_FLAGS "-Wl,--no-undefined")
catkin_package()

# if (CATKIN_ENABLE_TESTING)
#   find_package(roslaunch REQUIRED)
#   roslaunch_add_file_check(tests/roslaunch_test.xml)
# endif()

# install(DIRECTORY launch controller DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
