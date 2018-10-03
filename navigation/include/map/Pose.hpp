#pragma once

#include <cmath>
#include "Point.hpp"

/**
 * \class Pose   <-such as->  geometry_msgs::Pose2D
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

    bool operator == (const Pose &pose)
    {
        return this->point_ == pose.point_ && this->yaw_ == pose.yaw_ ? true : false;
    }

    inline void set(double x, double y, double yaw)
    {
        point_.setX(x);
        point_.setY(y);
        yaw_ = yaw;
    }
    inline void setPoint(Point p)  { point_ = p; }
    inline void setYaw(double yaw) { yaw_ = yaw; }

    inline const Point point(void) { return point_; }
    inline const double x(void)    { return point_.x(); }
    inline const double y(void)    { return point_.y(); }
    inline const double yaw(void)  { return yaw_; }

    inline double getDistance(Pose pose) const
    {
        return point_.getDistance(pose.point());
    }

  private:
    Point point_;
    double yaw_;
};
