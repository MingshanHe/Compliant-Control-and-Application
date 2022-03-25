#ifndef IMPEDACE_H
#define IMPEDACE_H

#include <ros/node_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_pinv_givens.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <realtime_tools/realtime_publisher.h>
#include "KDL_Base.h"
#include <kdl/chainidsolver_recursive_newton_euler.hpp>//Inverse Dynamics
#include <kdl/chaindynparam.hpp>                        //Inverse Dynamics Params

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/Dense"
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include "joint_effort_msg/JointEffort.h"
#include "joint_effort_msg/JointEfforts.h"
#include "joint_state_msg/JointState.h"
#include "geometry_msgs/WrenchStamped.h"
#include "std_msgs/Float64MultiArray.h"

typedef Matrix<double, 6, 6> Matrix6d;
class Impedance: public kdl_base::KDL_Base
{
public:
    Impedance() {}
    ~Impedance() {}

    void init(ros::NodeHandle &nh,
        std::string topic_arm_state,
        std::string topic_arm_command,
        std::string topic_wrench_state,
        std::vector<double> Ka,
        std::vector<double> Kv,
        std::vector<double> Kp,
        std::vector<double> M,
        std::vector<double> D,
        std::vector<double> K,
        std::vector<double> desired_pose);

    void run();

private:

    void compute_impedance(bool flag);

private:
    void state_arm_callback(const joint_state_msg::JointState msg);

    void state_wrench_callback(const geometry_msgs::WrenchStampedConstPtr msg);

    void command(const std_msgs::Float64MultiArray::ConstPtr &msg);

    void send_commands_to_robot();

protected:
    ros::NodeHandle                 nh_;
    // Subscribers:
    ros::Subscriber                 sub_command_; // Interface to external commands
    ros::Subscriber                 sub_arm_state_;
    ros::Subscriber                 sub_wrench_state_;
    ros::Subscriber                 sub_posture_;
    // Publishers:
    ros::Publisher                  pub_arm_cmd_;

    ros::Time                       last_publish_time_;
    double                          publish_rate_;

    // KDL Varaibales:
    KDL::JntArray                   Jnt_Pos_Init_State;
    KDL::JntArrayAcc                Jnt_Desired_State;
    KDL::JntArray                   CMD_State;
    KDL::JntArray                   Current_State;
    KDL::JntArray                   Jnt_Toq_Cmd_;

    KDL::JntArray                   Jnt_Pos_State;
    KDL::JntArray                   Jnt_Vel_State;
    KDL::JntArray                   Jnt_Toq_State;


    KDL::Wrenches                   Ext_Wrenches;

    KDL::Rotation                   Desired_Ori_;
    KDL::Vector                     Desired_Pos_;
    KDL::Frame                      Desired_Pose_;

    KDL::Vector                     Gravity;

    KDL::JntSpaceInertiaMatrix      M_; //Inertia Matrix
    KDL::JntArray                   C_, G_;
    KDL::JntArray                   Kp_, Kv_, Ka_;

    bool                            Recieved_Joint_State;
    bool                            Cmd_Flag_;
    bool                            Init_Flag_;
    uint                            Step_;

    //  Kinematics
    boost::shared_ptr<KDL::ChainFkSolverPos>    fk_pos_solver_;
    boost::shared_ptr<KDL::ChainFkSolverVel>    fk_vel_solver_;
    boost::shared_ptr<KDL::ChainIkSolverVel>    ik_vel_solver_;
    boost::shared_ptr<KDL::ChainIkSolverPos_NR> ik_pos_solver_;

    //  Dynamics
    boost::shared_ptr<KDL::ChainIdSolver>       id_pos_solver_;
    boost::shared_ptr<KDL::ChainDynParam>       id_solver_;

    std::vector<double>     Impedance_M, Impedance_D, Impedance_K;
    std::vector<double>     desired_pose_;

    double                  wrench_z;
    double                  pos_z;
};


#endif
