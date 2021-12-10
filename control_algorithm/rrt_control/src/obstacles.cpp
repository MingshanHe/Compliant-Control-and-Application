#include "rrt_control/obstacles.h"

Obstacles::Obstacles(
    const ros::NodeHandle &nh,
    const double    &check_step):
nh_(nh), check_step_(check_step)
{
}

/**
 * @brief Obstacles are stored as rectangles. Rectangle is denoted by two points : topLeft and bottomRight.
 * @param firstPoint
 * @param secondPoint
 */
void Obstacles::addObstacle(const vector<Vector3d> &points)
{
    obstacle = points;
}

/**
 * @brief Check if a line segment intersects a rectangle.
 * @param p1
 * @param p2
 * @return
 */
bool Obstacles::isSegmentInObstacle(const Vector3d point1,const Vector3d point2)
{   
    Vector3d point1_, point2_;
    if (point1.norm() < point2.norm())
    {
        point1_ = point1;
        point2_ = point2;
    }
    else
    {
        point1_ = point2;
        point2_ = point1;
    }
    Vector3d diff = point2_ - point1_;
    diff = (diff/diff.norm())*check_step_;
    while(point1_.norm() < point2_.norm() && ros::ok())
    {
        if(isPointInObstacle(point1_))
        {
            return true;
        }
        point1_ = point1_ + diff;
    }
}

bool Obstacles::isPointInObstacle(const Vector3d point)
{
    Vector3d delta_;
    for (size_t i = 0; i < 8; i++)
    {
        delta_ = point - obstacle.at(i);
    }
    for (size_t i = 0; i < 3; i++)
    {
        bool flag = false;
        bool flag_1;
        bool flag_2;
        bool in = false;
        for (size_t j = 0; j < 8; j++)
        {
            delta_ = point - obstacle.at(j);
            if(!flag)
            {
                if (delta_(i) > 0 ) {flag_1 = true;}
                else if (delta_(i) = 0) {return true;}
                else {flag_1 = false;}
                flag = true;
            }
            else
            {
                if (delta_(i) > 0 ) 
                {
                    flag_2 = true;
                    if(!(flag_1 && flag_2))
                    {
                        in = true;
                    }
                }
                else 
                {
                    flag_2 = false;
                    if(!(flag_1 && flag_2))
                    {
                        in = true;
                    }
                }
            }
        }
        if(!in)
        {
            return false;
        }
    }
    return true;
}