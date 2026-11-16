/*
 * @Author: MingshanHe
 * @Licence: MIT Licence
 */
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "admittance/admittance.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<admittance::Admittance>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("admittance"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
