#include "Communcation.hpp"

Communcation::Communcation()
{
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    cmdVelSub_ = nh.subscribe<msgs::CmdVel>("cmd_vel",
                                                     10,
                                                     boost::bind(&Communcation::cmdVelCallback, this, _1));
}

bool Communcation::start()
{
    setSendWay(CommunicateType::SERIAL);
    return true;
}

void Communcation::setSendWay(CommunicateType type)
{
    sendType_ = type;
}

void Communcation::cmdVelCallback(const msgs::CmdVel::ConstPtr &cmdVel)
{
    switch(sendType_)
    {
    case CommunicateType::SERIAL:
    {
        serialDataPack(cmdVel);
        ROS_DEBUG_STREAM("CommunicateType: Serial mode.");
        break;
    }

    case CommunicateType::UDP:
    {
        ROS_DEBUG_STREAM("CommunicateType: Udp stop.");
        break;
    }
    default:
        break;
    }

    dataSend();

    ROS_DEBUG("[CMD_VEL]");
}

void Communcation::dataSend()
{
    
    ROS_DEBUG("[dataSend]");
}

bool Communcation::serialDataPack(const msgs::CmdVel::ConstPtr &cmdVel)
{

}