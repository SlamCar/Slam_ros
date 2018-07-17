/** 
 * Copyright (c) 2018 old_wang Inc. All rights reserved. 
 */
#include "Communcation.hpp"
#include <ros/ros.h>

Communcation::Communcation()
{
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    cmdVelSub_ = nh.subscribe<cotek_msgs::CmdVel>("cmd_vel",
                                                     10,
                                                     boost::bind(&SwarmClient::cmdVelCallback, this, _1));
    
    taskPub_ = nh.advertise<cotek_msgs::TaskReq>("Task", 2);
    feedbackPub_ = nh.advertise<cotek_msgs::Feedback>("Feedback", 2);

    // handler_[TASK_REQ] = boost::bind(&SwarmClient::taskHandler, this, _1);
    // handler_[HEART_RSP] = boost::bind(&SwarmClient::heartbeatHandler, this, _1);
    // handler_[IPC_REQ] = boost::bind(&SwarmClient::ipcHandler, this, _1);
    // handler_[TASK_FINISH_RSP] = boost::bind(&SwarmClient::taskFinishHandler, this, _1);
    // handler_[ENTER_BUNDLE_RSP] = boost::bind(&SwarmClient::bundleHandler, this, _1);
    // handler_[CMD_SET_TIME] = boost::bind(&SwarmClient::timeHandler, this, _1);
    // handler_[CMD_GET_VERSION] = boost::bind(&SwarmClient::versionHandler, this, _1);
}

bool Communcation::start()
{
    // serverSocket_ = boost::make_shared<UdpSocket<UdpDataPack>>(serverIp_, serverPort_);
    // serverSocket_->listen(boost::bind(&SwarmClient::messageCallback, this, _1));
    // serverSocket_->run();

    // slaveSocket_ = boost::make_shared<UdpSocket<UdpDataPack>>(slaveIp_, slavePort_);
    // slaveSocket_->listen(boost::bind(&SwarmClient::messageCallback, this, _1));
    // slaveSocket_->run();

    return true;
}

void SwarmClient::cmdVelCallback(const cotek_msgs::CmdVel::ConstPtr &cmdVel)
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