#include "TourGuide.hpp"

#include <jsoncpp/json/json.h>
#include <ros/ros.h>
#include <ros/package.h>
// #include <sensor_msgs/LaserScan.h>
#include <fstream>


TourGuide::TourGuide() : sequence_(1), idx_(0)
{
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // private_nh.param("faketask_filepath", filename_, std::string("loopTask.json"));

    fakeTaskPub_ = nh.advertise<cotek_msgs::TaskReq>("Task", 2);
    // locationSub_ = nh.subscribe<cotek_msgs::Location>("Location", 2, boost::bind(&FakeServer::taskCallback, this, _1));
}