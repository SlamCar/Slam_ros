#pragma once

#include "msgs/CmdVel.h"

#include <ros/ros.h>

enum class CommunicateType : uint32_t
{
    SERIAL = 0,
};

class Communcation
{
public:

    static Communcation &getInstance()
    {
        static Communcation instance;
        return instance;
    }

    void dataSend(CommunicateType type);

    Communcation();

    virtual ~Communcation() {}

    bool start();

private:
    
    void cmdVelCallback(const msgs::CmdVel::ConstPtr &cmdVel);
    
    bool serialDataPack(const msgs::CmdVel::ConstPtr &cmdVel);

    void serialDataSend();
    
    ros::Subscriber cmdVelSub_;
    ros::Publisher feedbackPub_;
};