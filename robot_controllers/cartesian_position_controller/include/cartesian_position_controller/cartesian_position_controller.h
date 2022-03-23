/*
 * @Author: MingshanHe 
 * @Date: 2021-12-05 04:09:28 
 * @Last Modified by:   MingshanHe 
 * @Last Modified time: 2021-12-05 04:09:28 
 * @Licence: MIT Licence
 */
#ifndef CARTESIAN_POSITION_CONTROLLER_H
#define CARTESIAN_POSITION_CONTROLLER_H

#include <ros/node_handle.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <cartesian_state_msgs/PoseTwist.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_pinv_givens.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <realtime_tools/realtime_publisher.h>
#include "kinematics_base.h"

namespace cartesian_position_controller
{

/** \brief This class implements a ROS control cartesian velocity
 * controller. Its base class implements the core
 * of the controller.
 */
class Cartesian_Position_Controller: public kinematics_base::Kinematics_Base
{
public:
  Cartesian_Position_Controller() {}
  ~Cartesian_Position_Controller() {}

  /** \brief The init function is called to initialize the controller from a
   * non-realtime thread with a pointer to the hardware interface, itself,
   * instead of a pointer to a RobotHW.
   *s
   * \param robot The specific hardware interface used by this controller.
   *
   * \param n A NodeHandle in the namespace from which the controller
   * should read its configuration, and where it should set up its ROS
   * interface.
   *
   * \returns True if initialization was successful and the controller
   * is ready to be started.
   */
  bool init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n);

  /** \brief This is called from within the realtime thread just before the
   * first call to \ref update
   *
   * \param time The current time
   */
  void starting(const ros::Time& time);

  /*!
   * \brief Issues commands to the joint. Called at regular intervals
   */
  void update(const ros::Time& time, const ros::Duration& period);

  /*!
   * \brief Subscriber's callback function
   */
  void command_cart_pos(const geometry_msgs::PoseConstPtr &msg);
private:
  /** \brief Write current commands to the hardware interface
   */
  void writeVelocityCommands(const ros::Duration& period);
  void writePositionCommands(const ros::Duration& period);

protected:
  ros::Subscriber                 sub_command_; // Interface to external commands

  ros::Time                       last_publish_time_;
  double                          publish_rate_;
  double                          pre_e;

  KDL::Rotation                   End_Pos_Rotation;
  KDL::Vector                     End_Pos_Vector;
  KDL::JntArray                   Jnt_Vel_Cmd_;      // Desired joint velocity
  KDL::Twist                      End_Vel_Cmd_;      // Desired end-effector velocity
  KDL::JntArray                   Jnt_Pos_Cmd_;      // Desired joint position
  KDL::Frame                      End_Pos_Cmd_;      // Desired end-effector position
  KDL::FrameVel                   End_Vel_;
  KDL::Frame                      End_Pos_;
  cartesian_state_msgs::PoseTwist msg_state_;


  boost::shared_ptr<KDL::ChainFkSolverVel> fk_vel_solver_;
  boost::shared_ptr<KDL::ChainFkSolverPos> fk_pos_solver_;
  boost::shared_ptr<KDL::ChainIkSolverVel> ik_vel_solver_;
  boost::shared_ptr<KDL::ChainIkSolverPos> ik_pos_solver_;

  boost::shared_ptr<realtime_tools::RealtimePublisher<
     cartesian_state_msgs::PoseTwist> > realtime_pub_;
};
} // namespace controller_interface


#endif
