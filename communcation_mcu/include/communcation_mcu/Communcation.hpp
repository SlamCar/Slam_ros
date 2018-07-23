#pragma once

#include "msgs/CmdVel.h"
#include "Serial.hpp"
#include "SerialPack.hpp"

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

    bool init();

    void setSendWay(CommunicateType type);

    bool dataPack();

    void dataSend(const msgs::CmdVel::ConstPtr &cmdVel);

private:
    
    void cmdVelCallback(const msgs::CmdVel::ConstPtr &cmdVel);

    void serialDataSend(const msgs::CmdVel::ConstPtr &cmdVel);

    void udpDataSend(const msgs::CmdVel::ConstPtr &cmdVel);
    inline void setSendWay(CommunicateType type) {sendType_ = type;}
    
    ros::Subscriber cmdVelSub_;
    ros::Publisher feedbackPub_;
    CommunicateType sendType_;
    McuSerial ser_;
};