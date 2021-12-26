#include "rrt_control/rrt_control.h"
#include "ros/ros.h"
#include <geometry_msgs/Pose.h>

class rrt_control_node
{
public:
    rrt_control_node(ros::NodeHandle & nh);
    ~rrt_control_node(){}
    void run(ros::NodeHandle &nh);
private:
    ros::Publisher          pub_arm_cmd_;
    geometry_msgs::Pose     msg;

    std::vector<double>     desired_pose;
    std::string             topic_arm_command;

    std::vector<double>     workspace_limit;
    std::vector<Vector3d>   points;
    int                     max_iter;
    double                  step_size;
};