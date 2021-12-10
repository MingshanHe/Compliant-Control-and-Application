#ifndef OBSTACLES_H
#define OBSTACLES_H

#include <vector>
#include <eigen3/Eigen/Dense>
#include <assert.h>
// #include "constants.h"

#include "ros/ros.h"
using namespace Eigen;
using namespace std;


class Obstacles
{
public:
    Obstacles(
        const ros::NodeHandle &nh,
        const double          &check_step);

    void addObstacle(const vector<Vector3d> &points);

    bool isSegmentInObstacle(const Vector3d point1,const Vector3d point2);

    bool isPointInObstacle(const Vector3d point);


private:
    vector<Vector3d>    obstacle;
    double              check_step_;
    ros::NodeHandle     nh_;
};

#endif // OBSTACLES_H