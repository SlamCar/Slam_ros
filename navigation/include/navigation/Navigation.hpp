#pragma once

#include <ros/ros.h>
#include "Planner.hpp"
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <string>


/**
 * class Navigation: start an action server and calculate cmd_vel
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
     * \brief handle error
     * \return true if can continue, false otherwise
     */
    void handleError();

    boost::shared_ptr<TaskManager> tm_;
    boost::shared_ptr<StateManager> sm_;
    boost::shared_ptr<Planner> planner_;
    boost::shared_ptr<boost::thread> executor_;

    // publish velocity command
    ros::Publisher velPub_;

    double controllerFrequency_;
    std::string mapPackageName_;
    std::string mapFileName_;
};