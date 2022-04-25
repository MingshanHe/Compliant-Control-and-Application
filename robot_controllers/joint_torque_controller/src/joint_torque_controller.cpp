#include <pluginlib/class_list_macros.h>
#include "joint_torque_controller/kinematics_base.h"
#include "joint_torque_controller/joint_torque_controller.h"
#include "kdl_conversions/kdl_msg.h"

namespace joint_torque_controller
{
/** \brief Initialize the kinematic chain for kinematics-based computation.
 *
 */
bool Joint_Torque_Controller::init(
    hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n) {

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
    <joint_state_msg::JointState>(n, "joint_state", 1));

  // Topics
  sub_command_ = n.subscribe("command_joint_torque", 5,
    &Joint_Torque_Controller::command_joint_torque, this,
    ros::TransportHints().reliable().tcpNoDelay());

  // Variable init
  this->joint_state_.resize(this->kdl_chain_.getNrOfJoints());
  this->joint_effort_.resize(this->kdl_chain_.getNrOfJoints());
  Jnt_Vel_Cmd_.resize(this->kdl_chain_.getNrOfJoints());
  Jnt_Toq_Cmd_.resize(this->kdl_chain_.getNrOfJoints());
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
void Joint_Torque_Controller::starting(const ros::Time& time){
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
void Joint_Torque_Controller::update(const ros::Time& time, const ros::Duration& period) {
  // Get joint positions
    for(std::size_t i=0; i < this->joint_handles_.size(); i++)
    {
        this->joint_state_.q(i)     = this->joint_handles_[i].getPosition();
        this->joint_state_.qdot(i)  = this->joint_handles_[i].getVelocity();
        this->joint_effort_(i)      = this->joint_handles_[i].getEffort();
    }
  // Compute inverse kinematics velocity solver
//   ik_vel_solver_->CartToJnt(this->joint_state_.q, End_Vel_Cmd_, Jnt_Vel_Cmd_);
    writeTorqueCommands(period);

  // Forward kinematics
//   fk_vel_solver_->JntToCart(this->joint_state_, End_Vel_);
//   fk_pos_solver_->JntToCart(this->joint_state_.q, End_Pos_);

  // Limit rate of publishing
  if (publish_rate_ > 0.0 && last_publish_time_
        + ros::Duration(1.0/publish_rate_) < time) {

    // try to publish
    if (realtime_pub_->trylock()) {
      last_publish_time_ = last_publish_time_
                          + ros::Duration(1.0/publish_rate_);
      // populate message
      std::vector<double> pos_tmp, vel_tmp, eft_tmp;
      for (size_t i = 0; i < this->joint_handles_.size(); i++)
      {
        pos_tmp.push_back(this->joint_state_.q(i));
        vel_tmp.push_back(this->joint_state_.qdot(i));
        eft_tmp.push_back(this->joint_effort_(i));
      }

      realtime_pub_->msg_.position = pos_tmp;
      realtime_pub_->msg_.velocity = vel_tmp;
      realtime_pub_->msg_.effort   = eft_tmp;
      realtime_pub_->unlockAndPublish();
    }
  }
}

/*!
 * \brief Subscriber's callback: copies twist commands
 */
// void Joint_Torque_Controller::command_joint_torque(const joint_effort_msg::JointEffort &msg) {
//     for(std::size_t i=0; i < this->joint_handles_.size(); i++) {
//         Jnt_Toq_Cmd_(i) = msg.effort[i];
//     }
// }

void Joint_Torque_Controller::command_joint_torque(const joint_effort_msg::JointEfforts &msg) {
  Jnt_Toq_Cmd_(0) = msg.Joint1Effort;
  Jnt_Toq_Cmd_(1) = msg.Joint2Effort;
  Jnt_Toq_Cmd_(2) = msg.Joint3Effort;
  Jnt_Toq_Cmd_(3) = msg.Joint4Effort;
  Jnt_Toq_Cmd_(4) = msg.Joint5Effort;
  Jnt_Toq_Cmd_(5) = msg.Joint6Effort;
}

/********************************************/
/**FUNCTIONS OF INSTANCES OF THE BASE CLASS**/
/********************************************/

/** \brief write the desired velocity command in the hardware interface input
 * for a EffortJointInterface
 * \param period The duration of an update cycle
 */
void Joint_Torque_Controller::writeTorqueCommands(
                                    const ros::Duration& period) {
    for(std::size_t i=0; i < this->joint_handles_.size(); i++) {
        this->joint_handles_[i].setCommand(Jnt_Toq_Cmd_(i));
    }
}

} // controller_interface namespace

// Register controllers with the PLUGINLIB_EXPORT_CLASS macro to enable dynamic
// loading with the controller manager
PLUGINLIB_EXPORT_CLASS(joint_torque_controller::Joint_Torque_Controller, controller_interface::ControllerBase)
