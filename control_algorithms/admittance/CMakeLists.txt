cmake_minimum_required(VERSION 3.16)
project(admittance LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cartesian_state_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME} SHARED src/admittance.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${cartesian_state_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  Eigen3::Eigen
  rclcpp::rclcpp
  tf2::tf2
  tf2_eigen::tf2_eigen
  tf2_ros::tf2_ros
)

add_executable(${PROJECT_NAME}_node src/admittance_node.cpp)
target_link_libraries(${PROJECT_NAME}_node PRIVATE ${PROJECT_NAME})

add_executable(wrench_signal_generate src/wrench_signal_generate.cpp)
target_compile_features(wrench_signal_generate PRIVATE cxx_std_17)
target_link_libraries(wrench_signal_generate PRIVATE ${geometry_msgs_TARGETS} rclcpp::rclcpp)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS ${PROJECT_NAME}_node wrench_signal_generate DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(cartesian_state_msgs Eigen3 geometry_msgs rclcpp tf2 tf2_eigen tf2_ros)
ament_package()
