#include "rrt_control/rrt_control_node.h"
#include "ros/ros.h"
#include <geometry_msgs/Pose.h>

rrt_control_node::rrt_control_node(ros::NodeHandle & nh)
{
    if (!nh.getParam("desired_pose", desired_pose)) { ROS_ERROR("Couldn't retrieve the desired pose of the spring.");}
    if (!nh.getParam("topic_arm_command", topic_arm_command)) { ROS_ERROR("Couldn't retrieve the topic name for commanding the arm.");}

    pub_arm_cmd_ = nh.advertise<geometry_msgs::Pose>(topic_arm_command, 1);

    max_iter = 100000;
    step_size = 0.1;

    workspace_limit.push_back(-0.5);
    workspace_limit.push_back(0.5);
    workspace_limit.push_back(0.30);
    workspace_limit.push_back(0.5);
    workspace_limit.push_back(0.3);
    workspace_limit.push_back(0.75);
    // Vector3d p1,p2,p3,p4,p5,p6,p7,p8;
    // p1 << 0,0,0;
    // p2 << 1,0,0;
    // p3 << 1,1,0;
    // p4 << 0,1,0;
    // p5 << 0,0,1;
    // p6 << 1,0,1;
    // p7 << 1,1,1;
    // p8 << 0,1,1;
    // points.push_back(p1);
    // points.push_back(p2);
    // points.push_back(p3);
    // points.push_back(p4);
    // points.push_back(p5);
    // points.push_back(p6);
    // points.push_back(p7);
    // points.push_back(p8);
}

void rrt_control_node::run(ros::NodeHandle &nh)
{
    RRT rrt(nh, workspace_limit);
    // rrt.obstacles->addObstacle(points);
    rrt.setStepSize(step_size);
    rrt.setMaxIterations(max_iter);

    std::size_t i = 0;
    ros::Rate loop_rate_1(1);
    ros::Rate loop_rate(125);
    loop_rate_1.sleep();
    loop_rate_1.sleep();

    msg.position.x      = 0.0;
    msg.position.y      = 0.4;
    msg.position.z      = 0.5;
    msg.orientation.x   = 0.0;
    msg.orientation.y   = 0.707;
    msg.orientation.z   = 0.0;
    msg.orientation.w   = 0.707;
    pub_arm_cmd_.publish(msg);
    
    loop_rate_1.sleep();

    while(ros::ok())
    {
        Node *q = rrt.getRandomNode();
        if(q){
            Node *qNearest = rrt.nearest(q->position);
            if(rrt.distance(q->position, qNearest->position) > step_size){
                Vector3d newConfig = rrt.newConfig(q, qNearest);
                // if(! rrt.obstacles->isSegmentInObstacle(newConfig, qNearest->position)){
                {
                    Node *qNew = new Node;
                    qNew->position = newConfig;
                    rrt.add(qNearest, qNew);
                    geometry_msgs::Pose msg;
                    msg.position.x = qNew->position[0];
                    msg.position.y = qNew->position[1];
                    msg.position.z = qNew->position[2];
                    msg.orientation.x   = 0.0;
                    msg.orientation.y   = 0.707;
                    msg.orientation.z   = 0.0;
                    msg.orientation.w   = 0.707;
                    pub_arm_cmd_.publish(msg);
                    loop_rate.sleep();
                }
            }
            if (rrt.reached())
            {
                ROS_INFO("get reached.");
                break;
            }
        }

    }

    Node *q;
    if (rrt.reached())
    {
        q = rrt.lastNode;
    }
    else
    {
        q = rrt.nearest(rrt.endPos);
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "rrt_control_node");
    ros::NodeHandle nh;
    rrt_control_node rrt_control_node_(nh);
    rrt_control_node_.run(nh);
}