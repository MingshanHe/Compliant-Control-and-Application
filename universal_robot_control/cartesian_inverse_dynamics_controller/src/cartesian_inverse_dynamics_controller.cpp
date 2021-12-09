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
    dyn_param_solver_.reset(new KDL::ChainDynParam(this->kdl_chain_, this->gravity_));
    ee_jacobian_solver_.reset(new KDL::ChainJntToJacSolver(this->kdl_chain_));
    ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv_givens(this->kdl_chain_));
    fk_vel_solver_.reset(new KDL::ChainFkSolverVel_recursive(this->kdl_chain_));
    fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(this->kdl_chain_));

    wrench_wrist_       = KDL::Wrench();

    sub_command_ = n.subscribe ("/wrench",5,
                                &Cartesian_Inverse_Dynamics_Controller::command_cart_tau, this,
                                ros::TransportHints().reliable().tcpNoDelay());

    return true;
}

void Cartesian_Inverse_Dynamics_Controller::starting(const ros::Time& time)
{
    Jnt_Pos_State_.resize(this->kdl_chain_.getNrOfJoints());
    Jnt_Vel_State_.resize(this->kdl_chain_.getNrOfJoints());
    Jnt_Acc_State_.resize(this->kdl_chain_.getNrOfJoints());

    B.resize(this->kdl_chain_.getNrOfJoints());
    C.resize(this->kdl_chain_.getNrOfJoints());
    G.resize(this->kdl_chain_.getNrOfJoints());


    base_J_ee.resize(this->kdl_chain_.getNrOfJoints());
    last_publish_time_ = time;
    for(size_t i=0; i<kdl_chain_.getNrOfJoints(); i++)
    {
        Jnt_Vel_Past_State_.q(i)         = this->joint_handles_[i].getPosition();
        Jnt_Vel_Past_State_.qdot(i)      = this->joint_handles_[i].getVelocity();
    }
}

void Cartesian_Inverse_Dynamics_Controller::update(const ros::Time& time, const ros::Duration& period)
{
    // robot configuration
    // double dt = period.toSec();
    // Jnt_Acc_State_.data = (Jnt_Vel_State_.qdot.data - Jnt_Vel_Past_State_.qdot.data) / (dt);
    for(size_t i=0; i<kdl_chain_.getNrOfJoints(); i++)
    {
        Jnt_Pos_State_(i)           =   this->joint_handles_[i].getPosition();
        Jnt_Vel_State_.q(i)         =   this->joint_handles_[i].getPosition();
        Jnt_Vel_Past_State_.qdot(i) =   Jnt_Vel_State_.qdot(i);
        Jnt_Vel_State_.qdot(i)      =   this->joint_handles_[i].getVelocity();
    }
    // Joint Space Inertia Matrix B , Coriolis term C * q dot , Gravity G
    dyn_param_solver_->JntToMass(Jnt_Pos_State_, B);
    dyn_param_solver_->JntToCoriolis(Jnt_Vel_State_.q, Jnt_Vel_State_.qdot, C);
    dyn_param_solver_->JntToGravity(Jnt_Pos_State_, G);
    // Geometric Jacobians
    // ee_jacobian_solver_->JntToJac(Jnt_Pos_State_, base_J_ee);
    //Forward Kinematics
    // fk_pos_solver_->JntToCart(Jnt_Pos_State_, End_Pos_);

    // Jnt_Effort = B.data*Jnt_Acc_State_.data + C.data + G.data;

    writeTorqueCommands(period);

}
void Cartesian_Inverse_Dynamics_Controller::writeTorqueCommands(const ros::Duration &period)
{
    // this->joint_handles_[0].setCommand(Jnt_Effort(0));
    // this->joint_handles_[1].setCommand(Jnt_Effort(1));
    // this->joint_handles_[2].setCommand(Jnt_Effort(2));
    // this->joint_handles_[3].setCommand(Jnt_Effort(3));
    // this->joint_handles_[4].setCommand(Jnt_Effort(4));
    // this->joint_handles_[5].setCommand(Jnt_Effort(5));
    this->joint_handles_[0].setCommand(0.0);
    this->joint_handles_[1].setCommand(0.0);
    this->joint_handles_[2].setCommand(0.0);
    this->joint_handles_[3].setCommand(0.0);
    this->joint_handles_[4].setCommand(0.0);
    this->joint_handles_[5].setCommand(0.0);
}
void Cartesian_Inverse_Dynamics_Controller::command_cart_tau(const geometry_msgs::Wrench &msg)
{
    wrench_wrist_.force(0) = msg.force.x;
    wrench_wrist_.force(1) = msg.force.y;
    wrench_wrist_.force(2) = msg.force.z;
    wrench_wrist_.torque(0) = msg.torque.x;
    wrench_wrist_.torque(1) = msg.torque.y;
    wrench_wrist_.torque(2) = msg.torque.z;
}
}

PLUGINLIB_EXPORT_CLASS(cartesian_inverse_dynamics_controller::Cartesian_Inverse_Dynamics_Controller,
                       controller_interface::ControllerBase)