#include "Communcation.hpp"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "communcation_mcu_node");
    
    // Set ros log level:
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    if (!Communcation::getInstance().init())
    {
        ROS_ERROR("communcation_mcu_node init failed");
        return -1;
    }

    ROS_INFO_STREAM("MCU communcation started...");

    ros::spin();

    return 0;
}