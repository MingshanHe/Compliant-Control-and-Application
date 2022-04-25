#ifndef FILTER_CLASS_H
#define FILTER_CLASS_H

#include <iostream>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/Dense"

#include "ros/ros.h"

class Filter
{
public:
    Eigen::Vector3d LowPassFilter(const Eigen::Vector3d &data);
public:
    Filter(){};
    ~Filter(){};
    void init(const ros::NodeHandle &nh);
// LowPassFilter Params:
private:
    Eigen::Vector3d             Filter_K;
    Eigen::Vector3d             Filter_K_d;
    Eigen::Vector3d             LocalThres;
    Eigen::Vector3d             GlobalThres;
    Eigen::Vector3d             AddSum;
    Eigen::Vector3d             AddNum;
    Eigen::Vector3i             OldFlag;
    Eigen::Vector3i             NewFlag;

private:
    ros::NodeHandle             nh;
    Eigen::Vector3d             OldData;
    Eigen::Vector3d             NewData;
};

#endif