#include "Pose.hpp"
#include <ros/ros.h>
#include <gtest/gtest.h>

TEST(Pose, basics)
{
    {
        Pose p;
        EXPECT_DOUBLE_EQ(0.0, p.x());
        EXPECT_DOUBLE_EQ(0.0, p.y());
        EXPECT_DOUBLE_EQ(0.0, p.yaw());
        
        Pose p(1.0, 2.0, 3.0);
        EXPECT_DOUBLE_EQ(1.0, p.x());
        EXPECT_DOUBLE_EQ(2.0, p.y());
        EXPECT_DOUBLE_EQ(3.0, p.yaw());
    
        Point point(4.0, 5.0);
        Pose p(point, 6.0);
        EXPECT_DOUBLE_EQ(4.0, p.x());
        EXPECT_DOUBLE_EQ(5.0, p.y());
        EXPECT_DOUBLE_EQ(6.0, p.yaw());

        Pose p1(1.1, 2.2, 3.3);
        Pose p2 = p1;
        EXPECT_DOUBLE_EQ(1.1, p2.x());
        EXPECT_DOUBLE_EQ(2.2, p2.y());
        EXPECT_DOUBLE_EQ(3.3, p2.yaw());
 
        Pose p1(1.1, 2.2, 3.3);
        Pose p2 = p1;
        EXPECT_TRUE(p2 == p1);
    }
}