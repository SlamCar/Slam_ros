#include "Navigation.hpp"

Navigation::Navigation()
{
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.param("controller_frequency_", controllerFrequency_, 50.0);
    
    cmdVelPub_ = nh.advertise<msgs::CmdVel>("cmd_vel", 10);
    // feedBackSub_ = nh.subscribe<msgs::CmdVel>("feedback",
    //                                         10,
    //                                         boost::bind(&Navigation::feedbackCallback, this, _1));   

}

bool Navigation::init()
{
    return true;
}

void Navigation::run()
{

}

void Navigation::runner()
{
    // pidController_.dynamicAdjust();

    while (ros::ok())
    {
        ros::Rate rate(controllerFrequency_);



        if (rate.cycleTime() > ros::Duration(1.0 / controllerFrequency_))
        {
            ROS_WARN("Control loop missed its desired rate of %.2fHz... the heartbeat actually took %.2f seconds",
                     controllerFrequency_, rate.cycleTime().toSec());
        }
        
        // sleep to make sure the control frequency
        rate.sleep();
    }

    /*TODO
    ros::Timer controlling = n.createTimer(ros::Duration(1.0), callback, false);
    void callback(const ros::TimerEvent&)
    {
        ROS_INFO("Callback  triggered");
    }
    */
}

void dynamic_test(navigation::navigationConfig &config, uint32_t level)
{
    // msgs::CmdVel TestCmdVel; 
    // ros::Publisher cmdVel;
    // ROS_DEBUG("[ speed: %f ]",config.speed); 
    // TestCmdVel.driverVelocity = config.speed;
    // TestCmdVel.steeringAngle = 0.0;
    // cmdVel.publish(TestCmdVel);
}