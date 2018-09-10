#include "Point.hpp"
#include <ros/ros.h>
#include <gtest/gtest.h>

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}

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
        Point p1(0.0, 0.0);
        Point p2(1.0, 1.0);
        EXPECT_DOUBLE_EQ(1.413999999999, p1.getDistance(p2));
    }
}