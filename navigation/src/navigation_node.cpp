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

    dynamic_reconfigure::Server<navigation::navigationConfig> server;                     
    dynamic_reconfigure::Server<navigation::navigationConfig>::CallbackType f;
    f = boost::bind(&paramCallback, _1, _2);
    server.setCallback(f);
    
    Navigation navigation;

    if (navigation.init())
    {
        navigation.run();
        ros::spin();
    }

    return 0;
}