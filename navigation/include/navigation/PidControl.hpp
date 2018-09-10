#pragma once

#include "Controller.hpp"
#include "Pid.hpp"

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <navigation/navigationConfig.h>   //编译产生

//void paramCallback(navigation::navigationConfig &config, uint32_t level);

class PidController 
{
  public:
    PidController();
    
    inline void setP(double p) { p_ = p; }
    inline void setI(double i) { i_ = i; }
    inline void setD(double d) { d_ = d; }
    
    bool dynamicAdjust();

    double input();
    double output();
    
    
  private:
    double p_;
    double i_;
    double d_;
    MiniPID pid_;
    //boost::shared_ptr<boost::thread> adjust_;
};