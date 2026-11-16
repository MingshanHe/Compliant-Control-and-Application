/*
 * @Author: MingshanHe
 * @Licence: MIT Licence
 *
 * ROS 2 port of the hybrid position/force controller.
 */
#include "hybrid_position_force_control/hybrid_position_force_control.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>

#include <tf2_eigen/tf2_eigen.hpp>

namespace hybrid_position_force_control
{

using std::placeholders::_1;

HybridPositionForceControl::HybridPositionForceControl(const rclcpp::NodeOptions & options)
: Node("hybrid_position_force_control", options)
{
  // Topics
  const auto topic_arm_state = declare_parameter<std::string>(
    "topic_arm_state", "/cartesian_velocity_controller/ee_state");
  const auto topic_arm_command = declare_parameter<std::string>(
    "topic_arm_command", "/cartesian_velocity_controller/command_cart_vel");
  const auto topic_wrench_state = declare_parameter<std::string>(
    "topic_wrench_state", "/force_torque_sensor_broadcaster/wrench");
  const auto topic_cart_pose_cmd = declare_parameter<std::string>(
    "topic_cart_pose_cmd", "/desired_cartesian_pose_cmd");

  // Admittance parameters
  M_ = load_matrix("mass_arm");
  D_ = load_matrix("damping_arm");
  K_ = load_matrix("stiffness_coupling");
  frequency_ = declare_parameter<double>("frequency", 100.0);
  base_link_ = declare_parameter<std::string>("base_link", "base_link");
  end_link_ = declare_parameter<std::string>("end_link", "tool0");
  wrench_frame_ = declare_parameter<std::string>("wrench_frame", end_link_);
  const auto desired_pose = declare_parameter<std::vector<double>>(
    "desired_pose", std::vector<double>{});
  arm_max_vel_ = declare_parameter<double>("arm_max_vel", 1.5);
  arm_max_acc_ = declare_parameter<double>("arm_max_acc", 1.0);
  force_deadzone_thres_ = declare_parameter<double>("force_deadzone_thres", 0.0);
  torque_deadzone_thres_ = declare_parameter<double>("torque_deadzone_thres", 0.0);
  // Position loop: speed [m/s] of the reference towards a commanded position.
  approach_speed_ = declare_parameter<double>("approach_speed", 0.05);

  if (frequency_ <= 0.0) {
    throw std::invalid_argument("Parameter 'frequency' must be > 0");
  }
  if (approach_speed_ < 0.0) {
    throw std::invalid_argument("Parameter 'approach_speed' must be >= 0");
  }
  if (!desired_pose.empty() && desired_pose.size() != 7) {
    throw std::invalid_argument(
      "Parameter 'desired_pose' must have 7 elements [x y z qx qy qz qw]");
  }

  // Initialize the class variables
  arm_position_.setZero();
  arm_orientation_.setIdentity();
  arm_twist_.setZero();
  wrench_external_.setZero();
  force_desired_twist_adm_.setZero();
  arm_desired_acceleration_.setZero();
  error_.setZero();
  cmd_pose_desired_twist_adm_.setZero();
  cmd_pose_position_.setZero();
  cmd_pose_orientation_.setIdentity();
  desired_pose_position_.setZero();
  desired_pose_orientation_.setIdentity();
  if (desired_pose.size() == 7) {
    const Vector7d pose(desired_pose.data());
    desired_pose_position_ << pose.topRows(3);
    desired_pose_orientation_.coeffs() << pose.bottomRows(4) / pose.bottomRows(4).norm();
  }
  const bool use_initial_pose = desired_pose.empty();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Subscribers
  sub_arm_state_ = create_subscription<cartesian_state_msgs::msg::PoseTwist>(
    topic_arm_state, rclcpp::SystemDefaultsQoS(),
    [this, use_initial_pose](const cartesian_state_msgs::msg::PoseTwist::SharedPtr msg) {
      state_arm_callback(msg);
      if (use_initial_pose && !arm_state_ready_) {
        desired_pose_position_ = arm_position_;
        desired_pose_orientation_ = arm_orientation_;
        RCLCPP_INFO(
          get_logger(), "Desired pose taken from the current pose: [%.3f %.3f %.3f]",
          arm_position_.x(), arm_position_.y(), arm_position_.z());
      }
      arm_state_ready_ = true;
    });
  sub_wrench_state_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
    topic_wrench_state, rclcpp::SensorDataQoS(),
    std::bind(&HybridPositionForceControl::state_wrench_callback, this, _1));
  sub_cart_pose_cmd_ = create_subscription<geometry_msgs::msg::Pose>(
    topic_cart_pose_cmd, rclcpp::SystemDefaultsQoS(),
    std::bind(&HybridPositionForceControl::cart_pose_cmd_callback, this, _1));

  // Publishers
  pub_arm_cmd_ = create_publisher<geometry_msgs::msg::Twist>(
    topic_arm_command, rclcpp::SystemDefaultsQoS());

  // The timer follows the node clock, so it respects use_sim_time.
  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(1.0 / frequency_),
    std::bind(&HybridPositionForceControl::control_loop, this));

  RCLCPP_INFO(
    get_logger(), "Hybrid position/force control: state '%s', command '%s', wrench '%s', "
    "pose command '%s', %s -> %s @ %.0f Hz", topic_arm_state.c_str(), topic_arm_command.c_str(),
    topic_wrench_state.c_str(), topic_cart_pose_cmd.c_str(), base_link_.c_str(),
    end_link_.c_str(), frequency_);
}

Matrix6d HybridPositionForceControl::load_matrix(const std::string & name)
{
  const auto values = declare_parameter<std::vector<double>>(name, std::vector<double>{});
  if (values.size() != 36) {
    throw std::invalid_argument("Parameter '" + name + "' must be a 6x6 matrix (36 values)");
  }
  return Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(values.data());
}

//!-                    CONTROL LOOP                     -!//

void HybridPositionForceControl::control_loop()
{
  if (!arm_state_ready_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for the state of the arm...");
    return;
  }
  if (!ft_arm_ready_) {
    Matrix6d rot_matrix;
    if (!get_rotation_matrix(rot_matrix, base_link_, end_link_)) {
      return;
    }
    ft_arm_ready_ = true;
    RCLCPP_INFO(get_logger(), "The Force/Torque sensor is ready to use.");
    RCLCPP_INFO(get_logger(), "Running the hybrid position/force control loop .......");
  }

  compute_admittance();
  send_commands_to_robot();
}

//!-                Admittance Dynamics                  -!//

void HybridPositionForceControl::compute_admittance()
{
  const double dt = 1.0 / frequency_;

  // Position loop: the equilibrium of the admittance (spring anchor) moves
  // towards the commanded position at `approach_speed`, and the same velocity
  // is fed forward to the robot. In free space the end effector follows the
  // reference; in contact the anchor stops at the target, so the end effector
  // presses on the surface with F = K * (surface - target).
  // (The ROS 1 version kept the anchor at `desired_pose` and compared the target
  // with the measured position, so the spring stalled the robot D*v/K short of
  // the target and the tool never reached the work piece.)
  if (cmd_pose_flag_) {
    const double max_step = approach_speed_ * dt;
    for (int i = 0; i < 3; ++i) {
      const double step =
        std::clamp(cmd_pose_position_(i) - desired_pose_position_(i), -max_step, max_step);
      desired_pose_position_(i) += step;
      cmd_pose_desired_twist_adm_(i) = step / dt;
    }
  }

  error_.topRows(3) = arm_position_ - desired_pose_position_;
  if (desired_pose_orientation_.coeffs().dot(arm_orientation_.coeffs()) < 0.0) {
    arm_orientation_.coeffs() << -arm_orientation_.coeffs();
  }
  Eigen::Quaterniond quat_rot_err(arm_orientation_ * desired_pose_orientation_.inverse());
  if (quat_rot_err.coeffs().norm() > 1e-3) {
    quat_rot_err.coeffs() << quat_rot_err.coeffs() / quat_rot_err.coeffs().norm();
  }
  Eigen::AngleAxisd err_arm_des_orient(quat_rot_err);
  error_.bottomRows(3) << err_arm_des_orient.axis() * err_arm_des_orient.angle();

  // Translation error w.r.t. desired equilibrium
  Vector6d coupling_wrench_arm = D_ * force_desired_twist_adm_ + K_ * error_;
  arm_desired_acceleration_ = M_.inverse() * (-coupling_wrench_arm + wrench_external_);

  const double a_acc_norm = arm_desired_acceleration_.segment(0, 3).norm();
  if (a_acc_norm > arm_max_acc_) {
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Admittance generates high arm acceleration! norm: " << a_acc_norm);
    arm_desired_acceleration_.segment(0, 3) *= (arm_max_acc_ / a_acc_norm);
  }

  // Integrate for velocity based interface
  force_desired_twist_adm_ += arm_desired_acceleration_ * dt;
}

//!-                     CALLBACKS                       -!//

void HybridPositionForceControl::state_arm_callback(
  const cartesian_state_msgs::msg::PoseTwist::SharedPtr msg)
{
  arm_position_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  arm_orientation_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
    msg->pose.orientation.z, msg->pose.orientation.w;
  arm_twist_ << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z,
    msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z;
}

void HybridPositionForceControl::state_wrench_callback(
  const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
  if (!ft_arm_ready_) {
    return;
  }
  Vector6d wrench_ft_frame;
  wrench_ft_frame << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
    msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;
  for (int i = 0; i < 3; ++i) {
    if (std::abs(wrench_ft_frame(i)) < force_deadzone_thres_) {
      wrench_ft_frame(i) = 0.0;
    }
    if (std::abs(wrench_ft_frame(i + 3)) < torque_deadzone_thres_) {
      wrench_ft_frame(i + 3) = 0.0;
    }
  }

  const std::string & frame = msg->header.frame_id.empty() ? wrench_frame_ : msg->header.frame_id;
  Matrix6d rotation_ft_base;
  if (get_rotation_matrix(rotation_ft_base, base_link_, frame)) {
    wrench_external_ << rotation_ft_base * wrench_ft_frame;
  }
}

void HybridPositionForceControl::cart_pose_cmd_callback(
  const geometry_msgs::msg::Pose::SharedPtr msg)
{
  cmd_pose_position_ << msg->position.x, msg->position.y, msg->position.z;
  cmd_pose_orientation_.coeffs() << msg->orientation.x, msg->orientation.y,
    msg->orientation.z, msg->orientation.w;
  if (!cmd_pose_flag_) {
    // The reference starts from the current equilibrium (normally the desired pose).
    cmd_pose_flag_ = true;
  }
  RCLCPP_INFO(
    get_logger(), "New Cartesian position command: [%.3f %.3f %.3f]",
    cmd_pose_position_.x(), cmd_pose_position_.y(), cmd_pose_position_.z());
}

//!-               COMMANDING THE ROBOT                  -!//

void HybridPositionForceControl::send_commands_to_robot()
{
  const double norm_vel_des = force_desired_twist_adm_.segment(0, 3).norm();
  if (norm_vel_des > arm_max_vel_) {
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Admittance generates fast arm movements! velocity norm: " << norm_vel_des);
    force_desired_twist_adm_.segment(0, 3) *= (arm_max_vel_ / norm_vel_des);
  }

  const Vector6d twist = force_desired_twist_adm_ + cmd_pose_desired_twist_adm_;
  geometry_msgs::msg::Twist arm_twist_cmd;
  arm_twist_cmd.linear.x = twist(0);
  arm_twist_cmd.linear.y = twist(1);
  arm_twist_cmd.linear.z = twist(2);
  arm_twist_cmd.angular.x = twist(3);
  arm_twist_cmd.angular.y = twist(4);
  arm_twist_cmd.angular.z = twist(5);
  pub_arm_cmd_->publish(arm_twist_cmd);
}

//!-                    UTILIZATION                      -!//

bool HybridPositionForceControl::get_rotation_matrix(
  Matrix6d & rotation_matrix, const std::string & from_frame, const std::string & to_frame)
{
  try {
    const auto transform = tf_buffer_->lookupTransform(from_frame, to_frame, tf2::TimePointZero);
    const Eigen::Matrix3d rotation_from_to = tf2::transformToEigen(transform).linear();
    rotation_matrix.setZero();
    rotation_matrix.topLeftCorner(3, 3) = rotation_from_to;
    rotation_matrix.bottomRightCorner(3, 3) = rotation_from_to;
  } catch (const tf2::TransformException & ex) {
    rotation_matrix.setZero();
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Waiting for TF from: " << from_frame << " to: " << to_frame << " (" << ex.what() << ")");
    return false;
  }
  return true;
}

}  // namespace hybrid_position_force_control
