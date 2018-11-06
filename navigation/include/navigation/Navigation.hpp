#pragma once

#include "msgs/CmdVel.h"
#include "PidControl.hpp"

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <string>

void paramCallback(navigation::navigationConfig &config, uint32_t level)
{
    ROS_DEBUG("[ P: %f  I: %f  D: %f ]",config.P, config.I, config.D);  
    //pidController_.setP(config.P);
}


/**
 * class Navigation: use to calculate cmd_vel
 */
class Navigation
{
  public:
    /**
     * \brief initialize navigation node
     */
    Navigation();

    /**
     * \brief do initialization
     * \return true if init succeeded
     */
    bool init();

    /**
     * \brief start a thread and excute cycle
     */
    void run();


  private:
    /**
     * \brief excute the task send by action client
     * \param goal task goal which contains the whole navigation route
     */
    void runner();


    boost::shared_ptr<boost::thread> executor_;

    // publish velocity command
    ros::Publisher velPub_;
  
    boost::shared_ptr<boost::thread> test_;

    double controllerFrequency_;

    PidController pidController_;
};