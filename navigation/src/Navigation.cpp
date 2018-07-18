#include "Navigation.hpp"
#include "MapHelper.hpp"



Navigation::Navigation()
{
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.param("controller_frequency", controllerFrequency_, 50.0);
    velPub_ = nh.advertise<cotek_msgs::CmdVel>("cmd_vel", 10);

    // tm_ = boost::make_shared<TaskManager>();
    // sm_ = boost::make_shared<StateManager>(tm_);
}

bool Navigation::init()
{
    return MapHelper::getInstance().init(mapPackageName_, mapFileName_);
}

void Navigation::run()
{
    planner_ = boost::make_shared<Planner>();
    executor_ = boost::make_shared<boost::thread>(boost::bind(&Navigation::runner, this));
}

void Navigation::runner()
{
    while (ros::ok())
    {
        ros::Rate rate(controllerFrequency_);

        handleError();

        // planner_->update(tm_, sm_);
        // velPub_.publish(planner_->plan(tm_, sm_));
        velPub_publish();

        if (rate.cycleTime() > ros::Duration(1.0 / controllerFrequency_))
        {
            ROS_WARN("Control loop missed its desired rate of %.2fHz... the heartbeat actually took %.2f seconds",
                     controllerFrequency_, rate.cycleTime().toSec());
        }

        // sleep to make sure the control frequency
        rate.sleep();
    }
}