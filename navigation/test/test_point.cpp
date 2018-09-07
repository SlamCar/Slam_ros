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
        Point p;
        EXPECT_DOUBLE_EQ(0.0, p.x());
        EXPECT_DOUBLE_EQ(0.0, p.y());
    }
}