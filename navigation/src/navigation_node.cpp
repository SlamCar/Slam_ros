#include "Navigation.hpp"

#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "navigation_node");

    // Set ros log level:
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    Navigation navigation;

    if (navigation.init())
    {
        navigation.run();
        ros::spin();
    }

    return 0;
}