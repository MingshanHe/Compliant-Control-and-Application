/*
 * @Author: MingshanHe
 * @Licence: MIT Licence
 *
 * ROS 2 port of the hybrid position/force controller.
 *
 * A position loop moves the end effector towards a commanded Cartesian
 * position (constant-speed approach), while an admittance loop reacts to the
 * measured contact wrench so that the tool complies with the surface.
 */
#ifndef HYBRID_POSITION_FORCE_CONTROL__HYBRID_POSITION_FORCE_CONTROL_HPP_
#define HYBRID_POSITION_FORCE_CONTROL__HYBRID_POSITION_FORCE_CONTROL_HPP_

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "cartesian_state_msgs/msg/pose_twist.hpp"

namespace hybrid_position_force_control
{

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

class HybridPositionForceControl : public rclcpp::Node
{
public:
  explicit HybridPositionForceControl(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Control
  void control_loop();
  void compute_admittance();
  void send_commands_to_robot();

  // Callbacks
  void state_arm_callback(const cartesian_state_msgs::msg::PoseTwist::SharedPtr msg);
  void state_wrench_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
  void cart_pose_cmd_callback(const geometry_msgs::msg::Pose::SharedPtr msg);

  // Utilities
  bool get_rotation_matrix(
    Matrix6d & rotation_matrix, const std::string & from_frame,
    const std::string & to_frame);
  Matrix6d load_matrix(const std::string & name);

  // ROS interfaces
  rclcpp::Subscription<cartesian_state_msgs::msg::PoseTwist>::SharedPtr sub_arm_state_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_wrench_state_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_cart_pose_cmd_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_arm_cmd_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Parameters
  Matrix6d M_, D_, K_;
  double frequency_;
  double arm_max_vel_;
  double arm_max_acc_;
  double force_deadzone_thres_;
  double torque_deadzone_thres_;
  double approach_speed_;
  std::string base_link_;
  std::string end_link_;
  std::string wrench_frame_;

  // State
  Eigen::Vector3d arm_position_;
  Eigen::Quaterniond arm_orientation_;
  Vector6d arm_twist_;
  Vector6d wrench_external_;
  Vector6d force_desired_twist_adm_;
  Vector6d arm_desired_acceleration_;
  Vector6d error_;

  Eigen::Vector3d desired_pose_position_;
  Eigen::Quaterniond desired_pose_orientation_;
  Eigen::Vector3d cmd_pose_position_;
  Eigen::Quaterniond cmd_pose_orientation_;
  Vector6d cmd_pose_desired_twist_adm_;

  // Guards
  bool arm_state_ready_{false};
  bool ft_arm_ready_{false};
  bool cmd_pose_flag_{false};
};

}  // namespace hybrid_position_force_control

#endif  // HYBRID_POSITION_FORCE_CONTROL__HYBRID_POSITION_FORCE_CONTROL_HPP_
