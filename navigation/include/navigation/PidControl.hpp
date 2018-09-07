#pragma once

#include "Controller.hpp"
#include "Pid.hpp"

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <navigation/navigationConfig.h>   //编译产生

class PidController 
{
  public:
    PidController();
    void friend paramCallback(navigation::navigationConfig &config, uint32_t level);
    
  private:
    double p_;
    double i_;
    double d_;
    MiniPID pid_;
};