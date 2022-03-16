/*
 * @Author: MingshanHe 
 * @Date: 2021-12-05 04:08:00 
 * @Last Modified by: MingshanHe
 * @Last Modified time: 2021-12-05 04:08:21
 * @Licence: MIT Licence
 */

#ifndef HYBRID_POSITION_FORCE_CONTROL_H
#define HYBRID_POSITION_FORCE_CONTROL_H

#include "ros/ros.h"

#include "cartesian_state_msgs/PoseTwist.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/LaserScan.h"
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/Dense"
#include <eigen_conversions/eigen_msg.h>
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"

#include <memory>
#include <fstream>
#include <streambuf>
#include <iostream>
#include <cmath>


using namespace Eigen;

typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;
#define PI 3.1415926

class hybrid_position_force_control
{
protected:
  // ROS VARIABLES:
  ros::NodeHandle nh_;
  ros::Rate loop_rate_;

  // hybrid_position_force_control PARAMETERS:
  Matrix6d M_, D_, K_;

  // Subscribers:
  ros::Subscriber sub_arm_state_;
  ros::Subscriber sub_wrench_state_;
  ros::Subscriber sub_cart_pos_cmd_;
  // Publishers:
  ros::Publisher pub_arm_cmd_;

  // Variables:
  Vector3d      arm_position_;
  Quaterniond   arm_orientation_;
  Vector6d      arm_twist_;
  Vector6d      wrench_external_;
  Vector6d      force_desired_twist_adm_;
  Vector6d      arm_desired_accelaration;

  Vector7d      desired_pose_;
  Vector3d      desired_pose_position_;
  Quaterniond   desired_pose_orientation_;
  Vector7d      cmd_pose_;
  Vector3d      cmd_pose_position_;
  Quaterniond   cmd_pose_orientation_;
  Vector6d      cmd_pose_desired_twist_adm_;

  Vector6d      error;

  // TF:
  // Transform from base_link to world
  Matrix6d rotation_base_;
  // Listeners
  tf::TransformListener listener_ft_;
  tf::TransformListener listener_control_;
  tf::TransformListener listener_arm_;

  // Guards
  bool ft_arm_ready_;
  bool base_world_ready_;
  bool world_arm_ready_;
  bool cmd_pose_flag;

  double arm_max_vel_;
  double arm_max_acc_;

  double force_deadzone_thres_;
  double torque_deadzone_thres_;

public:
  hybrid_position_force_control(ros::NodeHandle &n, double frequency,
                      std::string topic_arm_state,
                      std::string topic_arm_command,
                      std::string topic_wrench_state,
                      std::string topic_cart_pose_cmd,
                      std::vector<double> M,
                      std::vector<double> D,
                      std::vector<double> K,
                      std::vector<double> desired_pose,
                      std::string base_link,
                      std::string end_link,
                      double arm_max_vel,
                      double arm_max_acc,
                      double force_deadzone_thres,
                      double torque_deadzone_thres
                       );
  ~hybrid_position_force_control(){}
  void run();
private:
  // Control
  void compute_admittance();
  // Callbacks
  void state_arm_callback(const cartesian_state_msgs::PoseTwistConstPtr msg);
  void state_wrench_callback(const geometry_msgs::WrenchStampedConstPtr msg);
  void cart_pose_cmd_callback(const geometry_msgs::Pose  msg);
  // Utilizations
  void send_commands_to_robot();
  void wait_for_transformations();
  bool get_rotation_matrix(Matrix6d & rotation_matrix,
                           tf::TransformListener & listener,
                           std::string from_frame,  std::string to_frame);
private:
  std::string   base_link_;
  std::string   end_link_;
};

#endif // HYBRID_POSITION_FORCE_CONTROL_H