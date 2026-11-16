/*
 * @Author: MingshanHe
 * @Licence: MIT Licence
 *
 * ROS 2 port of the Cartesian admittance controller.
 *
 *   M * a + D * v + K * (x - x_d) = F_ext
 *
 * The node integrates the admittance dynamics at a fixed rate and sends the
 * resulting end-effector twist to the Cartesian velocity controller.
 */
#ifndef ADMITTANCE__ADMITTANCE_HPP_
#define ADMITTANCE__ADMITTANCE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "cartesian_state_msgs/msg/pose_twist.hpp"

namespace admittance
{

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

class Admittance : public rclcpp::Node
{
public:
  explicit Admittance(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Control
  void control_loop();
  void compute_admittance();
  void send_commands_to_robot();

  // Callbacks
  void state_arm_callback(const cartesian_state_msgs::msg::PoseTwist::SharedPtr msg);
  void state_wrench_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);

  // Utilities
  bool get_rotation_matrix(
    Matrix6d & rotation_matrix, const std::string & from_frame,
    const std::string & to_frame);
  Matrix6d load_matrix(const std::string & name);

  // ROS interfaces
  rclcpp::Subscription<cartesian_state_msgs::msg::PoseTwist>::SharedPtr sub_arm_state_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_wrench_state_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_arm_cmd_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Admittance parameters
  Matrix6d M_, D_, K_;
  double frequency_;
  double arm_max_vel_;
  double arm_max_acc_;
  double force_deadzone_thres_;
  double torque_deadzone_thres_;
  std::string base_link_;
  std::string end_link_;
  std::string wrench_frame_;

  // State
  Eigen::Vector3d arm_position_;
  Eigen::Quaterniond arm_orientation_;
  Vector6d arm_twist_;
  Vector6d wrench_external_;
  Vector6d arm_desired_twist_adm_;
  Vector6d arm_desired_acceleration_;
  Vector6d error_;

  Eigen::Vector3d desired_pose_position_;
  Eigen::Quaterniond desired_pose_orientation_;

  // Guards
  bool arm_state_ready_{false};
  bool ft_arm_ready_{false};
};

}  // namespace admittance

#endif  // ADMITTANCE__ADMITTANCE_HPP_
