/*
 * @Author: MingshanHe
 * @Licence: MIT Licence
 *
 * ROS 2 (ros2_control) port of the Cartesian velocity controller.
 *
 * The controller receives an end-effector twist (expressed in `root_name`) on
 * `~/command_cart_vel`, maps it to joint velocities with a KDL pseudo-inverse
 * velocity IK solver and writes either joint velocities or integrated joint
 * positions to the hardware. The end-effector pose and twist are published on
 * `~/ee_state` (cartesian_state_msgs/msg/PoseTwist).
 *
 * The kinematic chain is built from the robot description that the controller
 * manager hands to every controller, so the same plugin works for any serial
 * arm (UR5, Franka, B601-DM, ...).
 */
#ifndef CARTESIAN_VELOCITY_CONTROLLER__CARTESIAN_VELOCITY_CONTROLLER_HPP_
#define CARTESIAN_VELOCITY_CONTROLLER__CARTESIAN_VELOCITY_CONTROLLER_HPP_

#include <array>
#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <realtime_tools/realtime_thread_safe_box.hpp>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainiksolvervel_pinv_givens.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>

#include "cartesian_state_msgs/msg/pose_twist.hpp"

namespace cartesian_velocity_controller
{

class CartesianVelocityController : public controller_interface::ControllerInterface
{
public:
  CartesianVelocityController() = default;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  /// Twist command written by the subscriber, read by the real-time loop.
  struct TwistCommand
  {
    std::array<double, 6> values{};  // vx, vy, vz, wx, wy, wz (root frame)
    uint64_t sequence{0};            // incremented on every received message
  };

  bool build_kinematics(const std::string & robot_description);
  void write_commands(const rclcpp::Duration & period);
  void hold_position();

  // Parameters
  std::string root_name_;
  std::string tip_name_;
  std::string command_interface_type_;  // "velocity" or "position"
  double publish_rate_{125.0};
  double command_timeout_{0.5};
  double velocity_scaling_{1.0};
  double max_joint_velocity_{0.0};

  // Kinematics
  KDL::Chain kdl_chain_;
  std::vector<std::string> joint_names_;
  std::vector<double> joint_velocity_limits_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
  std::unique_ptr<KDL::ChainFkSolverVel_recursive> fk_vel_solver_;
  std::unique_ptr<KDL::ChainIkSolverVel_pinv_givens> ik_vel_solver_;

  // Loaned interfaces, ordered like joint_names_
  template<typename T>
  using InterfaceRefs = std::vector<std::reference_wrapper<T>>;
  InterfaceRefs<hardware_interface::LoanedStateInterface> joint_position_handles_;
  InterfaceRefs<hardware_interface::LoanedStateInterface> joint_velocity_handles_;
  InterfaceRefs<hardware_interface::LoanedCommandInterface> joint_command_handles_;

  // Real-time state
  KDL::JntArrayVel joint_state_;
  KDL::JntArray joint_velocity_cmd_;
  KDL::JntArray joint_position_cmd_;
  KDL::Twist ee_twist_cmd_;
  KDL::Frame ee_pose_;
  KDL::FrameVel ee_frame_vel_;
  uint64_t last_sequence_{0};
  rclcpp::Time last_command_time_;
  rclcpp::Time last_publish_time_;
  bool has_command_{false};
  bool first_update_{true};

  // ROS interfaces
  realtime_tools::RealtimeThreadSafeBox<TwistCommand> command_box_;
  std::atomic<uint64_t> command_sequence_{0};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<cartesian_state_msgs::msg::PoseTwist>>
  rt_state_pub_;
  cartesian_state_msgs::msg::PoseTwist state_msg_;
};

}  // namespace cartesian_velocity_controller

#endif  // CARTESIAN_VELOCITY_CONTROLLER__CARTESIAN_VELOCITY_CONTROLLER_HPP_
