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

    if (!ros::master::check())
    {
        ROS_ERROR("Please start roscore first.");
        return -1;
    }

    ROS_ASSERT_MSG(Communcation::getInstance().init(),"communcation_mcu_node init failed");

    Communcation::getInstance().run();
    
    ros::spin();

    return 0;
}