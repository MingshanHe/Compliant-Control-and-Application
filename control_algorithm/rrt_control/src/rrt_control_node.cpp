#include "rrt_control/rrt_control.h"
#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "rrt_control_node");
    ros::NodeHandle nh;
    ros::Publisher Pose_pub = nh.advertise<geometry_msgs::Pose>("Pose",1);
    std::vector<double> workspace_limit;
    int max_iter = 300000;
    double step_size = 2.0;
    workspace_limit.push_back(-2);
    workspace_limit.push_back(0);
    workspace_limit.push_back(-2);
    workspace_limit.push_back(0);
    workspace_limit.push_back(2);
    workspace_limit.push_back(2);
    
    vector<Vector3d> points;
    Vector3d p1,p2,p3,p4,p5,p6,p7,p8;
    p1 << 0,0,0;
    p2 << 1,0,0;
    p3 << 1,1,0;
    p4 << 0,1,0;
    p5 << 0,0,1;
    p6 << 1,0,1;
    p7 << 1,1,1;
    p8 << 0,1,1;
    points.push_back(p1);
    points.push_back(p2);
    points.push_back(p3);
    points.push_back(p4);
    points.push_back(p5);
    points.push_back(p6);
    points.push_back(p7);
    points.push_back(p8);

    RRT rrt(nh, workspace_limit);
    rrt.obstacles->addObstacle(points);
    rrt.setStepSize(step_size);
    rrt.setMaxIterations(max_iter);

    std::size_t i = 0;
    ros::Rate loop_rate(10);
    while(ros::ok() && i<max_iter)
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
                    // geometry_msgs::Pose msg;
                    // msg.position.x = qNew->position[0];
                    // msg.position.y = qNew->position[1];
                    // msg.position.z = qNew->position[2];
                    // Pose_pub.publish(msg);
                    // loop_rate.sleep();
                    printf("%f, %f, %f", newConfig[0], newConfig[1], newConfig[2]);
                    ROS_INFO("Publish.");
                }
            }
        }
        if (rrt.reached())
        {
            ROS_INFO("get reached.");
            break;
        }
        i++;
        // printf("i: %d\n",i);
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