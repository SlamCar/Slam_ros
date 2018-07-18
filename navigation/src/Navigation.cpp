#include "Navigation.hpp"


Navigation::Navigation()
{
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.param("controller_frequency_", controllerFrequency_, 50.0);
    private_nh.param("test_driverVelocity_", test_driverVelocity_, 2.0);
    private_nh.param("test_steeringAngle_", test_steeringAngle_, 0.0);
    
    velPub_ = nh.advertise<msgs::CmdVel>("cmd_vel", 10);

    // tm_ = boost::make_shared<TaskManager>();
    // sm_ = boost::make_shared<StateManager>(tm_);
}

bool Navigation::init()
{
    test_switch(true);
    return true;
    // return MapHelper::getInstance().init(mapPackageName_, mapFileName_);
}

void Navigation::run()
{
    // planner_ = boost::make_shared<Planner>();

    if(test_flag_)
    {
        test_ = boost::make_shared<boost::thread>(boost::bind(&Navigation::test_runner, this));
    }
    else
    {
       executor_ = boost::make_shared<boost::thread>(boost::bind(&Navigation::runner, this)); 
    }
}

void Navigation::runner()
{
    while (ros::ok())
    {
        ros::Rate rate(controllerFrequency_);

        // planner_->update(tm_, sm_);
        // velPub_.publish(planner_->plan(tm_, sm_));
        // velPub_.publish();

        if (rate.cycleTime() > ros::Duration(1.0 / controllerFrequency_))
        {
            ROS_WARN("Control loop missed its desired rate of %.2fHz... the heartbeat actually took %.2f seconds",
                     controllerFrequency_, rate.cycleTime().toSec());
        }

        // sleep to make sure the control frequency
        rate.sleep();
    }
}

void Navigation::test_switch(bool flag)
{
    if(flag)
    {
        test_flag_ = true;
    }
    else
    {
        test_flag_ = false;
    }
}


void Navigation::test_runner()
{
    ROS_DEBUG("-----test_runner-----");
    msgs::CmdVel TestCmdVel;

    while (ros::ok())
    { 
        ros::Rate rate(controllerFrequency_); 
        TestCmdVel.driverVelocity = test_driverVelocity_;
        TestCmdVel.steeringAngle = test_steeringAngle_;
        velPub_.publish(TestCmdVel);

        if (rate.cycleTime() > ros::Duration(1.0 / controllerFrequency_))
        {
            ROS_WARN("Control loop missed its desired rate of %.2fHz... the heartbeat actually took %.2f seconds",
                     controllerFrequency_, rate.cycleTime().toSec());
        }
    }
}