#include "Point.hpp"

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <cmath>


TEST(Point, basics)
{
    {
        Point p1;
        EXPECT_DOUBLE_EQ(0.0, p1.x());
        EXPECT_DOUBLE_EQ(0.0, p1.y());
    
        Point p2(1.0, 2.0);
        EXPECT_DOUBLE_EQ(1.0, p2.x());
        EXPECT_DOUBLE_EQ(2.0, p2.y());
    
        Point p3(p2);
        EXPECT_DOUBLE_EQ(1.0, p2.x());
        EXPECT_DOUBLE_EQ(2.0, p2.y());

        Point p4 = p2;
        EXPECT_DOUBLE_EQ(1.0, p2.x());
        EXPECT_DOUBLE_EQ(2.0, p2.y());
    }

    {
        Point p;
        p.setX(1.1);
        p.setY(2.2);
        EXPECT_DOUBLE_EQ(1.1, p.x());
        EXPECT_DOUBLE_EQ(2.2, p.y());

        p.setXY(2.2, 1.1);
        EXPECT_DOUBLE_EQ(2.2, p.x());
        EXPECT_DOUBLE_EQ(1.1, p.y());
    }   

    {
        Point p1(1.0, 1.0);
        Point p2(1.0, 1.0);
        EXPECT_TRUE( p1 == p2 );
    }

    {
        Point p1(0.0, 0.0);
        Point p2(1.0, 1.0);
        EXPECT_DOUBLE_EQ(sqrt(2), p1.getDistance(p2));
    }
}