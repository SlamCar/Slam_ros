#include "PidControl.hpp"


void paramCallback(navigation::navigationConfig &config, uint32_t level)
{
    ROS_DEBUG("[ P: %f  I: %f  D: %f ]",config.P, config.I, config.D);
}

PidController::PidController() : pid_(0.0, 0.0, 0.0)
{
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~"); 

    private_nh.param("P", p_, 3.075);
    private_nh.param("I", i_, 0.05);
    private_nh.param("D", d_, 0.6);
}

bool PidController::dynamicAdjust()
{
    /*dynamic config*/
    dynamic_reconfigure::Server<navigation::navigationConfig> server;
    dynamic_reconfigure::Server<navigation::navigationConfig>::CallbackType f;
    f = boost::bind(&paramCallback, _1, _2);
    server.setCallback(f);
    return true;
}