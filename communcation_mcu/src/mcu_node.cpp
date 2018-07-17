/** 
 * Copyright (c) 2018 old_wang Inc. All rights reserved. 
 */
#include "Communcation.hpp"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mcu_node");

    // Set ros log level:
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    if (!Communcation::getInstance().start())
    {
        return -1;
    }

    ROS_INFO_STREAM("MCU communcation started...");

    ros::spin();

    return 0;
}