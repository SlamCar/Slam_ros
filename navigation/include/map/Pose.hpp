#pragma once

#include <cmath>
#include "Point.hpp"

/**
 * \class Pose
 */
class Pose
{
  public:
    Pose() : point_(0.0, 0.0), yaw_(0.0) {}
    Pose(double x, double y, double yaw) : point_(x,y), yaw_(yaw) {}
    Pose(Point p, double yaw) : point_(p), yaw_(yaw) {}

    Pose &operator = (const Pose pose)
    {
        if (this == &pose)
        {
            return *this;
        }
        this -> point_ = pose.point_;
        this -> yaw_ = pose.yaw_;
        return *this;
    } 

    bool &operator == (const Pose pose)
    {
        
        if (this == &pose)
    
        return *this;
    }

  private:
    Point point_;
    double yaw_;
}
