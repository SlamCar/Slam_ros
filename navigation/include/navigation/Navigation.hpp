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

    /**
     * \brief test switch open or close 
     */
    void test_switch(bool flag);
    
    /**
     * \brief start a test task
     */
    void test_runner();

    /**
     * \brief handle error
     * \return true if can continue, false otherwise
     */
    void handleError();

    // boost::shared_ptr<TaskManager> tm_;
    // boost::shared_ptr<StateManager> sm_;
    // boost::shared_ptr<Planner> planner_;
    boost::shared_ptr<boost::thread> executor_;

    // publish velocity command
    ros::Publisher velPub_;

    //test cmd_vel
    bool test_flag_;
    double test_driverVelocity_;
    double test_steeringAngle_;
  
    boost::shared_ptr<boost::thread> test_;

    double controllerFrequency_;
    std::string mapPackageName_;
    std::string mapFileName_;

    PidController pidController_;
};