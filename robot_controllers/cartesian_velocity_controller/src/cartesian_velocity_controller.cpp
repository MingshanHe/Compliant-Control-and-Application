/*
 * @Author: MingshanHe
 * @Licence: MIT Licence
 *
 * ROS 2 (ros2_control) port of the Cartesian velocity controller.
 */
#include "cartesian_velocity_controller/cartesian_velocity_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <controller_interface/helpers.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <urdf/model.hpp>

namespace cartesian_velocity_controller
{

using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using controller_interface::interface_configuration_type;

CallbackReturn CartesianVelocityController::on_init()
{
  try {
    // Kinematic chain (root -> tip) taken from the robot description.
    auto_declare<std::string>("root_name", "base_link");
    auto_declare<std::string>("tip_name", "tool0");
    // Optional: if given, it must match the movable joints of the chain (in order).
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});
    // Hardware command interface: "velocity" (default) or "position" (integrated).
    auto_declare<std::string>("command_interface", hardware_interface::HW_IF_VELOCITY);
    // Rate [Hz] of the ~/ee_state publisher (<= 0 disables it).
    auto_declare<double>("publish_rate", 125.0);
    // A command older than this [s] is replaced by a zero twist (<= 0 disables).
    auto_declare<double>("command_timeout", 0.5);
    // Joint velocity commands are scaled so that |qdot_i| <= scaling * limit_i, where
    // limit_i is the URDF velocity limit, capped at max_joint_velocity if that is > 0.
    auto_declare<double>("velocity_scaling", 1.0);
    auto_declare<double>("max_joint_velocity", 0.0);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during init: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration CartesianVelocityController::command_interface_configuration() const
{
  InterfaceConfiguration config{interface_configuration_type::INDIVIDUAL, {}};
  for (const auto & joint : joint_names_) {
    config.names.push_back(joint + "/" + command_interface_type_);
  }
  return config;
}

InterfaceConfiguration CartesianVelocityController::state_interface_configuration() const
{
  InterfaceConfiguration config{interface_configuration_type::INDIVIDUAL, {}};
  for (const auto & joint : joint_names_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return config;
}

bool CartesianVelocityController::build_kinematics(const std::string & robot_description)
{
  const auto logger = get_node()->get_logger();

  urdf::Model model;
  if (robot_description.empty() || !model.initString(robot_description)) {
    RCLCPP_ERROR(logger, "Failed to parse the robot description (URDF).");
    return false;
  }

  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(model, kdl_tree)) {
    RCLCPP_ERROR(logger, "Failed to construct the KDL tree.");
    return false;
  }

  kdl_chain_ = KDL::Chain();
  if (!kdl_tree.getChain(root_name_, tip_name_, kdl_chain_)) {
    RCLCPP_ERROR(
      logger, "Failed to get a KDL chain from '%s' to '%s'. The tree has %u joints and these "
      "segments:", root_name_.c_str(), tip_name_.c_str(), kdl_tree.getNrOfJoints());
    for (const auto & segment : kdl_tree.getSegments()) {
      RCLCPP_ERROR(logger, "  %s", segment.first.c_str());
    }
    return false;
  }

  joint_names_.clear();
  joint_velocity_limits_.clear();
  for (const auto & segment : kdl_chain_.segments) {
    const auto & joint = segment.getJoint();
    if (joint.getType() == KDL::Joint::Fixed) {
      continue;
    }
    joint_names_.push_back(joint.getName());
    double limit = std::numeric_limits<double>::infinity();
    const auto urdf_joint = model.getJoint(joint.getName());
    if (urdf_joint && urdf_joint->limits && urdf_joint->limits->velocity > 0.0) {
      limit = urdf_joint->limits->velocity;
    }
    if (max_joint_velocity_ > 0.0) {
      limit = std::min(limit, max_joint_velocity_);
    }
    joint_velocity_limits_.push_back(limit);
  }

  RCLCPP_INFO(
    logger, "Kinematic chain %s -> %s: %u segments, %u joints", root_name_.c_str(),
    tip_name_.c_str(), kdl_chain_.getNrOfSegments(), kdl_chain_.getNrOfJoints());
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    RCLCPP_INFO(
      logger, "  joint[%zu] %s (velocity limit %.3f rad/s)", i, joint_names_[i].c_str(),
      joint_velocity_limits_[i]);
  }

  fk_pos_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
  fk_vel_solver_ = std::make_unique<KDL::ChainFkSolverVel_recursive>(kdl_chain_);
  ik_vel_solver_ = std::make_unique<KDL::ChainIkSolverVel_pinv_givens>(kdl_chain_);
  return true;
}

CallbackReturn CartesianVelocityController::on_configure(const rclcpp_lifecycle::State &)
{
  const auto node = get_node();
  const auto logger = node->get_logger();

  root_name_ = node->get_parameter("root_name").as_string();
  tip_name_ = node->get_parameter("tip_name").as_string();
  command_interface_type_ = node->get_parameter("command_interface").as_string();
  publish_rate_ = node->get_parameter("publish_rate").as_double();
  command_timeout_ = node->get_parameter("command_timeout").as_double();
  velocity_scaling_ = node->get_parameter("velocity_scaling").as_double();
  max_joint_velocity_ = node->get_parameter("max_joint_velocity").as_double();

  if (command_interface_type_ != hardware_interface::HW_IF_VELOCITY &&
    command_interface_type_ != hardware_interface::HW_IF_POSITION)
  {
    RCLCPP_ERROR(
      logger, "Parameter 'command_interface' must be 'velocity' or 'position', got '%s'.",
      command_interface_type_.c_str());
    return CallbackReturn::ERROR;
  }
  if (velocity_scaling_ <= 0.0) {
    RCLCPP_ERROR(logger, "Parameter 'velocity_scaling' must be > 0.");
    return CallbackReturn::ERROR;
  }

  if (!build_kinematics(get_robot_description())) {
    return CallbackReturn::ERROR;
  }

  const auto expected = node->get_parameter("joints").as_string_array();
  if (!expected.empty() && expected != joint_names_) {
    RCLCPP_ERROR(
      logger, "Parameter 'joints' does not match the movable joints of the chain %s -> %s.",
      root_name_.c_str(), tip_name_.c_str());
    return CallbackReturn::ERROR;
  }

  const unsigned int n = kdl_chain_.getNrOfJoints();
  joint_state_.resize(n);
  joint_velocity_cmd_.resize(n);
  joint_position_cmd_.resize(n);
  joint_position_handles_.reserve(n);
  joint_velocity_handles_.reserve(n);
  joint_command_handles_.reserve(n);

  command_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
    "~/command_cart_vel", rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
      TwistCommand cmd;
      cmd.values = {msg->linear.x, msg->linear.y, msg->linear.z,
        msg->angular.x, msg->angular.y, msg->angular.z};
      cmd.sequence = ++command_sequence_;
      command_box_.set(cmd);
    });

  rt_state_pub_ =
    std::make_unique<realtime_tools::RealtimePublisher<cartesian_state_msgs::msg::PoseTwist>>(
    node, "~/ee_state", rclcpp::SystemDefaultsQoS());
  state_msg_.header.frame_id = root_name_;
  state_msg_.child_frame_id = tip_name_;

  RCLCPP_INFO(
    logger, "Configured: command interface '%s', publish rate %.1f Hz, command timeout %.2f s",
    command_interface_type_.c_str(), publish_rate_, command_timeout_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianVelocityController::on_activate(const rclcpp_lifecycle::State &)
{
  // The controller manager does not guarantee the order of the loaned
  // interfaces, so look them up by joint name.
  joint_position_handles_.clear();
  joint_velocity_handles_.clear();
  joint_command_handles_.clear();
  if (!controller_interface::get_ordered_interfaces(
      state_interfaces_, joint_names_, hardware_interface::HW_IF_POSITION,
      joint_position_handles_) ||
    !controller_interface::get_ordered_interfaces(
      state_interfaces_, joint_names_, hardware_interface::HW_IF_VELOCITY,
      joint_velocity_handles_) ||
    !controller_interface::get_ordered_interfaces(
      command_interfaces_, joint_names_, command_interface_type_, joint_command_handles_))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not find all the joint interfaces.");
    return CallbackReturn::ERROR;
  }

  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    const auto q = joint_position_handles_[i].get().get_optional();
    if (!q) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Could not read the position of joint %s.",
        joint_names_[i].c_str());
      return CallbackReturn::ERROR;
    }
    joint_state_.q(i) = *q;
    joint_state_.qdot(i) = 0.0;
    joint_position_cmd_(i) = *q;
    joint_velocity_cmd_(i) = 0.0;
  }

  // Discard any command received while inactive.
  command_box_.set(TwistCommand{});
  last_sequence_ = 0;
  command_sequence_ = 0;
  has_command_ = false;
  ee_twist_cmd_ = KDL::Twist::Zero();
  // Timestamps are initialized from the controller-manager clock in update().
  first_update_ = true;
  hold_position();
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianVelocityController::on_deactivate(const rclcpp_lifecycle::State &)
{
  joint_velocity_cmd_.data.setZero();
  hold_position();
  joint_position_handles_.clear();
  joint_velocity_handles_.clear();
  joint_command_handles_.clear();
  return CallbackReturn::SUCCESS;
}

void CartesianVelocityController::hold_position()
{
  for (std::size_t i = 0; i < joint_command_handles_.size(); ++i) {
    const double value = command_interface_type_ == hardware_interface::HW_IF_VELOCITY ?
      0.0 : joint_position_cmd_(i);
    (void)joint_command_handles_[i].get().set_value(value);
  }
}

controller_interface::return_type CartesianVelocityController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (first_update_) {
    last_publish_time_ = time;
    last_command_time_ = time;
    first_update_ = false;
  }

  // Read joint state.
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    const auto q = joint_position_handles_[i].get().get_optional();
    const auto qdot = joint_velocity_handles_[i].get().get_optional();
    if (q) {joint_state_.q(i) = *q;}
    if (qdot) {joint_state_.qdot(i) = *qdot;}
  }

  // Fetch the latest twist command (non-blocking).
  if (const auto cmd = command_box_.try_get()) {
    if (cmd->sequence != last_sequence_ && cmd->sequence != 0) {
      last_sequence_ = cmd->sequence;
      last_command_time_ = time;
      has_command_ = true;
      for (unsigned int k = 0; k < 3; ++k) {
        ee_twist_cmd_.vel(k) = cmd->values[k];
        ee_twist_cmd_.rot(k) = cmd->values[k + 3];
      }
    }
  }
  if (has_command_ && command_timeout_ > 0.0 &&
    (time - last_command_time_).seconds() > command_timeout_)
  {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "No Cartesian velocity command for %.2f s, stopping.", command_timeout_);
    ee_twist_cmd_ = KDL::Twist::Zero();
    has_command_ = false;
  }

  // Velocity IK: twist (root frame, reference point at tip) -> joint velocities.
  if (ik_vel_solver_->CartToJnt(joint_state_.q, ee_twist_cmd_, joint_velocity_cmd_) < 0 ||
    !joint_velocity_cmd_.data.allFinite())
  {
    joint_velocity_cmd_.data.setZero();
  }

  // Scale the whole vector to respect the joint velocity limits (keeps the direction).
  double scale = 1.0;
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    const double limit = velocity_scaling_ * joint_velocity_limits_[i];
    if (std::isfinite(limit) && limit > 0.0) {
      scale = std::max(scale, std::abs(joint_velocity_cmd_(i)) / limit);
    }
  }
  if (scale > 1.0) {
    joint_velocity_cmd_.data /= scale;
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "Joint velocity limits reached, scaling the command by %.3f", 1.0 / scale);
  }

  write_commands(period);

  // Forward kinematics and state publication.
  fk_pos_solver_->JntToCart(joint_state_.q, ee_pose_);
  fk_vel_solver_->JntToCart(joint_state_, ee_frame_vel_);

  if (publish_rate_ > 0.0 && (time - last_publish_time_).seconds() >= 1.0 / publish_rate_) {
    last_publish_time_ = time;
    state_msg_.header.stamp = time;
    state_msg_.pose.position.x = ee_pose_.p.x();
    state_msg_.pose.position.y = ee_pose_.p.y();
    state_msg_.pose.position.z = ee_pose_.p.z();
    ee_pose_.M.GetQuaternion(
      state_msg_.pose.orientation.x, state_msg_.pose.orientation.y,
      state_msg_.pose.orientation.z, state_msg_.pose.orientation.w);
    const KDL::Twist twist = ee_frame_vel_.GetTwist();
    state_msg_.twist.linear.x = twist.vel.x();
    state_msg_.twist.linear.y = twist.vel.y();
    state_msg_.twist.linear.z = twist.vel.z();
    state_msg_.twist.angular.x = twist.rot.x();
    state_msg_.twist.angular.y = twist.rot.y();
    state_msg_.twist.angular.z = twist.rot.z();
    rt_state_pub_->try_publish(state_msg_);
  }

  return controller_interface::return_type::OK;
}

void CartesianVelocityController::write_commands(const rclcpp::Duration & period)
{
  const double dt = period.seconds();
  for (std::size_t i = 0; i < joint_command_handles_.size(); ++i) {
    auto & command = joint_command_handles_[i].get();
    if (command_interface_type_ == hardware_interface::HW_IF_VELOCITY) {
      (void)command.set_value(joint_velocity_cmd_(i));
    } else {
      // Integrate on the previous command (not on the measurement) so that the
      // position reference stays smooth on real hardware.
      joint_position_cmd_(i) += joint_velocity_cmd_(i) * dt;
      (void)command.set_value(joint_position_cmd_(i));
    }
  }
}

}  // namespace cartesian_velocity_controller

PLUGINLIB_EXPORT_CLASS(
  cartesian_velocity_controller::CartesianVelocityController,
  controller_interface::ControllerInterface)
