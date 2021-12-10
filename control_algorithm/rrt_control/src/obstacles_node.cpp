#include "rrt_control/obstacles.h"
#include "ros/ros.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacles_node_test");
    ros::NodeHandle nh;
    double check_step = 0.01;
    Obstacles obstacle_(nh, check_step);
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
    obstacle_.addObstacle(points);
    bool check_segment;
    Vector3d point1, point2;
    point1 << 0, 0, 2;
    point2 << 2, 2, 2;
    check_segment = obstacle_.isSegmentInObstacle(point1, point2);
    if (check_segment)
    {
        ROS_INFO("In obstacle.");
    }
    else
    {
        ROS_INFO("Not In obstacle.");
    }
        
}