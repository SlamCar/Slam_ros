#pragma once

#include "msgs/CmdVel.h"

#include <ros/ros.h>

enum class CommunicateType : uint32_t
{
    SERIAL = 0,
    UDP
};

class Communcation
{
public:

    static Communcation &getInstance()
    {
        static Communcation instance;
        return instance;
    }
    
    Communcation();

    virtual ~Communcation() {}

    bool start();

    void setSendWay(CommunicateType type);

    void dataPack();

    void dataSend();

private:
    
    void cmdVelCallback(const msgs::CmdVel::ConstPtr &cmdVel);
    
    bool serialDataPack(const msgs::CmdVel::ConstPtr &cmdVel);

    bool udpDataPack(const msgs::CmdVel::ConstPtr &cmdVel);
    
    ros::Subscriber cmdVelSub_;
    ros::Publisher feedbackPub_;
    CommunicateType sendType_;
};