#include "Communcation.hpp"
#include <ros/ros.h>

Communcation::Communcation()
{
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    cmdVelSub_ = nh.subscribe<cotek_msgs::CmdVel>("cmd_vel",
                                                     10,
                                                     boost::bind(&Communcation::cmdVelCallback, this, _1));
    
    taskPub_ = nh.advertise<cotek_msgs::TaskReq>("Task", 2);
    feedbackPub_ = nh.advertise<cotek_msgs::Feedback>("Feedback", 2);

}

bool Communcation::start()
{
    
    return true;
}

void Communcation::cmdVelCallback(const msgs::CmdVel::ConstPtr &cmdVel)
{
    IpcRequest req;
    req.command = cmdVel->command;
    req.velocity = cmdVel->velocity;
    req.omega = cmdVel->omega;
    req.driverVelocity = cmdVel->driverVelocity;
    req.steeringAngle = cmdVel->steeringAngle;
    req.abnormal = cmdVel->abnormal;
    req.avoidStrategy = cmdVel->avoidStrategy;
    req.mlsDirection = cmdVel->mlsDirection;

    send2Slave(IPC_RSP, reinterpret_cast<uint8_t *>(&req), sizeof(IpcRequest));
    ROS_DEBUG("[CMD_VEL]");
}