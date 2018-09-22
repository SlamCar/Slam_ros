#pragma once

#include <cmath>

/**
 * \class Point
 */
class Point 
{
  public:
    
    Point() : x_(0.0), y_(0.0) {}
    Point(double x, double y) : x_(x), y_(y) {}
    Point(const Point &p) : x_(p.x_), y_(p.y_) {}
    
    Point &operator = (const Point &p)
    {
        if (this == &p)
        {
            return *this;
        }
        this->x_ = p.x_;
        this->y_ = p.y_;
        return *this;
    }

    bool operator == (const Point &p)
    {
        return this->x_ == p.x_ && this->y_ == p.y_ ? true : false;
        // if(this->x_ == p.x_ && this->y_ == p.y_)
        //     return true;
        // else 
        //     return false;
    }

    inline void setX(double x) { x_ = x; }
    inline void setY(double y) { y_ = y; }
    inline void setXY(double x, double y) { x_ = x, y_ = y; }

    inline const double x(void) { return x_; }
    inline const double y(void) { return y_; }

    inline const double getDistance(Point point) const
    {
        return std::hypot(point.x() - x_, point.y() - y_);
    }   

  private:
    double x_;
    double y_;

};