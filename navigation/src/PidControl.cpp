#include "PidControl.hpp"


PidController::PidController() : pid_(0.0, 0.0, 0.0)
{
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.param("P", p_, 3.075);
    private_nh.param("I", i_, 0.05);
    private_nh.param("D", d_, 0.6);

    pid_.setPID(p_, i_, d_);
}

bool PidController::dynamicAdjust()
{
    /*dynamic config*/
    /**
     * 作用域在此函数内
     */
    // dynamic_reconfigure::Server<navigation::navigationConfig> server;                     
    // dynamic_reconfigure::Server<navigation::navigationConfig>::CallbackType f;
    // f = boost::bind(&paramCallback, _1, _2);
    // server.setCallback(f);
    // while(ros::ok())
    // {
       
    // }

    // return true;
}