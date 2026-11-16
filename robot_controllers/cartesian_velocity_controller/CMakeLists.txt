cmake_minimum_required(VERSION 3.16)
project(cartesian_velocity_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  cartesian_state_msgs
  controller_interface
  geometry_msgs
  hardware_interface
  kdl_parser
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  urdf
)

find_package(ament_cmake REQUIRED)
find_package(orocos_kdl REQUIRED)
foreach(dep IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dep} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED src/cartesian_velocity_controller.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${cartesian_state_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  kdl_parser::kdl_parser
  orocos-kdl
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
  urdf::urdf
)

pluginlib_export_plugin_description_file(controller_interface cartesian_velocity_controller_plugin.xml)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} orocos_kdl)
ament_package()
