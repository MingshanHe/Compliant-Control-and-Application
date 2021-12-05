/*
 * @Author: MingshanHe 
 * @Date: 2021-12-05 04:09:41 
 * @Last Modified by: MingshanHe
 * @Last Modified time: 2021-12-05 04:18:32
 * @Licence: MIT Licence
 */
#include <pluginlib/class_list_macros.h>
#include "cartesian_velocity_controller/kinematics_base.h"
#include "cartesian_velocity_controller/cartesian_velocity_controller.h"
#include "kdl_conversions/kdl_msg.h"

namespace cartesian_velocity_controller
{
/** \brief Initialize the kinematic chain for kinematics-based computation.
 *
 */
bool Cartesian_Velocity_Controller::init(
    hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n) {

  // KDL
  kinematics_base::Kinematics_Base::init(robot, n);

  ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv_givens(this->kdl_chain_));
  fk_vel_solver_.reset(new KDL::ChainFkSolverVel_recursive(this->kdl_chain_));
  fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(this->kdl_chain_));

  // get publishing period
  if (!n.getParam("publish_rate", publish_rate_)){
      ROS_ERROR("Parameter 'publish_rate' not set");
      return false;
  }
  realtime_pub_.reset(new realtime_tools::RealtimePublisher
    <cartesian_state_msgs::PoseTwist>(n, "ee_state", 4));


  // Topics
  sub_command_ = n.subscribe("command_cart_vel", 5,
    &Cartesian_Velocity_Controller::command_cart_vel, this,
    ros::TransportHints().reliable().tcpNoDelay());

  // Variable init
  this->joint_state_.resize(this->kdl_chain_.getNrOfJoints());
  this->joint_effort_.resize(this->kdl_chain_.getNrOfJoints());
  Jnt_Vel_Cmd_.resize(this->kdl_chain_.getNrOfJoints());
  End_Vel_Cmd_ = KDL::Twist::Zero();
  End_Pos_.p.Zero();
  End_Pos_.M.Identity();
  End_Vel_.p.Zero();
  End_Vel_.M.Identity();

  return true;
}

/** \brief This is called from within the realtime thread just before the
 * first call to \ref update
 *
 * \param time The current time
 */
void Cartesian_Velocity_Controller::starting(const ros::Time& time){
  for(std::size_t i=0; i < this->joint_handles_.size(); i++) {
    Jnt_Vel_Cmd_(i) = 0.0;
    this->joint_state_.q(i)     = 0.0;
    this->joint_state_.qdot(i)  = 0.0;
    this->joint_effort_(i)    = 0.0;
  }
  End_Vel_Cmd_ = KDL::Twist::Zero();
  last_publish_time_ = time;
}

/*!
 * \brief Issues commands to the joint. Should be called at regular intervals
 */
void Cartesian_Velocity_Controller::update(const ros::Time& time, const ros::Duration& period) {
  // Get joint positions
  for(std::size_t i=0; i < this->joint_handles_.size(); i++)
  {
    // if(fabs(this->joint_handles_[i].getPosition()-(this->joint_state_.q(i) + Jnt_Vel_Cmd_(i)*period.toSec()))<0.0000001 || Jnt_Vel_Cmd_(i)!=0)
    // {
      this->joint_state_.q(i)         = this->joint_handles_[i].getPosition();
      this->joint_state_.qdot(i)      = this->joint_handles_[i].getVelocity();
      this->joint_effort_(i)        = this->joint_handles_[i].getEffort();
    // }
  }
  // Compute inverse kinematics velocity solver
  ik_vel_solver_->CartToJnt(this->joint_state_.q, End_Vel_Cmd_, Jnt_Vel_Cmd_);
  writeVelocityCommands(period);

  // Forward kinematics
  fk_vel_solver_->JntToCart(this->joint_state_, End_Vel_);
  fk_pos_solver_->JntToCart(this->joint_state_.q, End_Pos_);

  // Limit rate of publishing
  if (publish_rate_ > 0.0 && last_publish_time_
       + ros::Duration(1.0/publish_rate_) < time) {

    // try to publish
    if (realtime_pub_->trylock()) {
      last_publish_time_ = last_publish_time_
                           + ros::Duration(1.0/publish_rate_);
      // populate message
      realtime_pub_->msg_.header.stamp = time;
      tf::poseKDLToMsg(End_Pos_, realtime_pub_->msg_.pose);
      tf::twistKDLToMsg(End_Vel_.GetTwist(), realtime_pub_->msg_.twist);

      realtime_pub_->unlockAndPublish();
    }
  }
}

/*!
 * \brief Subscriber's callback: copies twist commands
 */
void Cartesian_Velocity_Controller::command_cart_vel(const geometry_msgs::TwistConstPtr &msg) {
    End_Vel_Cmd_.vel(0) = msg->linear.x;
    End_Vel_Cmd_.vel(1) = msg->linear.y;
    End_Vel_Cmd_.vel(2) = msg->linear.z;
    End_Vel_Cmd_.rot(0) = msg->angular.x;
    End_Vel_Cmd_.rot(1) = msg->angular.y;
    End_Vel_Cmd_.rot(2) = msg->angular.z;
}


/********************************************/
/**FUNCTIONS OF INSTANCES OF THE BASE CLASS**/
/********************************************/

/** \brief write the desired velocity command in the hardware interface input
 * for a VelocityJointInterface
 * \param period The duration of an update cycle
 */
void Cartesian_Velocity_Controller::writeVelocityCommands(
                                    const ros::Duration& period) {
    for(std::size_t i=0; i < this->joint_handles_.size(); i++) {
      this->joint_handles_[i].setCommand(this->joint_state_.q(i)
                                      + Jnt_Vel_Cmd_(i)*period.toSec());
    }
}

} // controller_interface namespace

// Register controllers with the PLUGINLIB_EXPORT_CLASS macro to enable dynamic
// loading with the controller manager
PLUGINLIB_EXPORT_CLASS(cartesian_velocity_controller::Cartesian_Velocity_Controller,
                       controller_interface::ControllerBase)
