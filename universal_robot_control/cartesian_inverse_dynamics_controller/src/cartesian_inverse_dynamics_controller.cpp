#include <pluginlib/class_list_macros.h>
#include "cartesian_inverse_dynamics_controller/kinematics_base.h"
#include "cartesian_inverse_dynamics_controller/cartesian_inverse_dynamics_controller.h"

namespace cartesian_inverse_dynamics_controller
{

bool Cartesian_Inverse_Dynamics_Controller::init(
    hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n
)
{
    kinematics_base::Kinematics_Base::init(robot, n);
    ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv_givens(this->kdl_chain_));
    fk_vel_solver_.reset(new KDL::ChainFkSolverVel_recursive(this->kdl_chain_));
    fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(this->kdl_chain_));

  // get publishing period
    // if (!n.getParam("publish_rate", publish_rate_)){
    //     ROS_ERROR("Parameter 'publish_rate' not set");
    //     return false;
    // }
//   realtime_pub_.reset(new realtime_tools::RealtimePublisher
//     <cartesian_state_msgs::PoseTwist>(n, "ee_state", 4));


    // Topics
    // sub_command_ = n.subscribe("command_cart_vel", 5,
    //     &Cartesian_Velocity_Controller::command_cart_vel, this,
    //     ros::TransportHints().reliable().tcpNoDelay());

    // Variable init
    // this->joint_state_.resize(this->kdl_chain_.getNrOfJoints());
    // this->joint_effort_.resize(this->kdl_chain_.getNrOfJoints());
    // Jnt_Vel_Cmd_.resize(this->kdl_chain_.getNrOfJoints());
    // End_Vel_Cmd_ = KDL::Twist::Zero();
    // End_Pos_.p.Zero();
    // End_Pos_.M.Identity();
    // End_Vel_.p.Zero();
    // End_Vel_.M.Identity();

    return true;
}

void Cartesian_Inverse_Dynamics_Controller::starting(const ros::Time& time)
{
    last_publish_time_ = time;
}

void Cartesian_Inverse_Dynamics_Controller::update(const ros::Time& time, const ros::Duration& period)
{
    // for(std::size_t i=0; i < this->joint_handles_.size(); i++)
    // {
    //     // if(fabs(this->joint_handles_[i].getPosition()-(this->joint_state_.q(i) + Jnt_Vel_Cmd_(i)*period.toSec()))<0.0000001 || Jnt_Vel_Cmd_(i)!=0)
    //     // {
    //     this->joint_state_.q(i)         = this->joint_handles_[i].getPosition();
    //     this->joint_state_.qdot(i)      = this->joint_handles_[i].getVelocity();
    //     this->joint_effort_(i)        = this->joint_handles_[i].getEffort();
    //     // }
    // }
    // // Compute inverse kinematics velocity solver
    // ik_vel_solver_->CartToJnt(this->joint_state_.q, End_Vel_Cmd_, Jnt_Vel_Cmd_);
    // writeVelocityCommands(period);

    // // Forward kinematics
    // fk_vel_solver_->JntToCart(this->joint_state_, End_Vel_);
    // fk_pos_solver_->JntToCart(this->joint_state_.q, End_Pos_);

    // // Limit rate of publishing
    // if (publish_rate_ > 0.0 && last_publish_time_
    //     + ros::Duration(1.0/publish_rate_) < time) {

    //     // try to publish
    //     if (realtime_pub_->trylock()) {
    //     last_publish_time_ = last_publish_time_
    //                         + ros::Duration(1.0/publish_rate_);
    //     // populate message
    //     realtime_pub_->msg_.header.stamp = time;
    //     tf::poseKDLToMsg(End_Pos_, realtime_pub_->msg_.pose);
    //     tf::twistKDLToMsg(End_Vel_.GetTwist(), realtime_pub_->msg_.twist);

    //     realtime_pub_->unlockAndPublish();
    //     }
    // }
}
}

PLUGINLIB_EXPORT_CLASS(cartesian_inverse_dynamics_controller::Cartesian_Inverse_Dynamics_Controller,
                       controller_interface::ControllerBase)