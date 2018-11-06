#pragma once
#include <ros/ros.h>

class RobotConfig
{
  public:
    void init();
    void setConfig();

    std::string cmd_vel_topic_;
  
  private:

};