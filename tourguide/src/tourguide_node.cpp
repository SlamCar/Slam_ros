#include "TourGuide.hpp"

#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tourguide_node");

    // Set ros log level:
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    TourGuide tourguide;

    if (!tourguide.init())
    {
        return -1;
    }

    ros::spin();

    return 0;
}
