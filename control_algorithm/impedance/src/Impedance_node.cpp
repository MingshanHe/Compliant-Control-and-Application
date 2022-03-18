#include "ros/ros.h"
#include "impedance/Impedance.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "impedance_node");

    ros::NodeHandle nh;


    // Parameters
    double frequency;

    std::string topic_arm_state;
    std::string topic_arm_command;
    std::string topic_wrench_state;

    std::vector<double> desired_pose;
    std::vector<double> Ka, Kv, Kp;



    // LOADING PARAMETERS FROM THE ROS SERVER 

    // Topic names
    if (!nh.getParam("topic_arm_state", topic_arm_state)) { ROS_ERROR("Couldn't retrieve the topic name for the state of the arm."); return -1; }
    if (!nh.getParam("topic_arm_command", topic_arm_command)) { ROS_ERROR("Couldn't retrieve the topic name for commanding the arm."); return -1; }
    if (!nh.getParam("topic_wrench_state", topic_wrench_state)) { ROS_ERROR("Couldn't retrieve the topic name for the force/torque sensor."); return -1; }

    if (!nh.getParam("Ka", Ka)) { ROS_ERROR("Couldn't retrieve the Ka."); return -1; }
    if (!nh.getParam("Kv", Kv)) { ROS_ERROR("Couldn't retrieve the Kv."); return -1; }
    if (!nh.getParam("Kp", Kp)) { ROS_ERROR("Couldn't retrieve the Kp."); return -1; }

    if (!nh.getParam("desired_pose", desired_pose)) { ROS_ERROR("Couldn't retrieve the desired_pose."); return -1; }

    Impedance impedance;

    impedance.init(
        nh,
        topic_arm_state,
        topic_arm_command,
        topic_wrench_state,
        Ka, Kv, Kp,
        desired_pose);

    impedance.run();

    return 0;
}