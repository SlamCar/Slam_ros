#pragma once

#include "msgs/TaskReq.h"

#include <ros/ros.h>
#include <map>
#include <vector>
#include <string>

class TourGuide
{
  public:
    TourGuide();
    ~TourGuide() {}

    bool init();
    void run();
    // void taskCallback(const cotek_msgs::Location::ConstPtr &loc);

  private:
    int idx_;
    ros::Publisher TaskPub_;
  //  ros::Subscriber locationSub_;
    std::string filename_;
    std::uint32_t sequence_;
    std::map<uint32_t, cotek_msgs::Node> nodeMap_;
    std::vector<std::vector<uint32_t>> taskList_;
    msgs::TaskReq taskReq_;
};
