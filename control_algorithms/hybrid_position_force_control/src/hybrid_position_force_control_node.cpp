/*
 * @Author: MingshanHe
 * @Licence: MIT Licence
 */
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "hybrid_position_force_control/hybrid_position_force_control.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
      std::make_shared<hybrid_position_force_control::HybridPositionForceControl>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("hybrid_position_force_control"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
