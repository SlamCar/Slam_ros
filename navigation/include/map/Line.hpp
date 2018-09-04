#pragma once
#include "Point.hpp"
#include "Pose.hpp"

#include <camth>

/**
 * \class Line
 * \brief a line is described as y = kx + b in world-coodinate system
 */
class Line
{
  public:
    Line(double a,double b,double c) : a_(a),b_(b),c_(c) {}
    Line(const Point& pt1, double angle)
    {

    }
    Line(const Point& pt1, const Point& pt2)
    {

    }

    inline void setA(double a) { a_ = a; }
    inline void setB(double b) { b_ = b; }
    inline void setC(double c) { c_ = c; }

  private:
    /**
     * Line equation: ax + by + c = 0;
     */
    double a_;
    double b_;
    double c_;

};