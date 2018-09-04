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
        {   
            return *this;
        }
        
        this->point_ = pose.point_;
        this->yaw_ = pose.yaw_;
        return *this;
    }

    inline set(double x, double y, double yaw)
    {
        point_.setX(x);
        point_.setY(y);
        yaw_ = yaw;
    }
    inline setPoint(Point p)  { point_ = p; }
    inline setYaw(doubel yaw) { yaw_ = yaw; }

    inline const Point point(void) { return point_; }
    inline const double x(void)    { return point_.x(); }
    inline const double y(void)    { return point_.y(); }
    inline const double yaw(void)  { return yaw_; }

    inline double getDistance(const Pose pose) const
    {
        return point_.getDistance(pose.point_());
    }

  private:
    Point point_;
    double yaw_;
}
