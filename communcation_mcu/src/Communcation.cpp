#include "Communcation.hpp"
#include "SerialPack.hpp"

Communcation::Communcation()
{
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    cmdVelSub_ = nh.subscribe<msgs::CmdVel>("cmd_vel",
                                                     10,
                                                     boost::bind(&Communcation::cmdVelCallback, this, _1));
}

bool Communcation::init()
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
    switch(sendType_)
    {
    case CommunicateType::SERIAL:
    {
        ROS_DEBUG_STREAM("CommunicateType: Serial mode.");
        serialDataSend();
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

}

void Communcation::serialDataPack(const msgs::CmdVel::ConstPtr &cmdVel)
{
    TaskRequest task;
    task.driverVelocity = cmdVel->driverVelocity;
    task.steeringAngle = cmdVel->steeringAngle;

    SerialPack pack(CmdType::TASK_REQ);
    pack.setLen(20);
    pack.setBody(reinterpret_cast<uint8_t *>(&task), pack.len());
    pack.generateCrc();
}

void Communcation::serialDataSend()
{
    // McuSerial_.connect();
    ROS_DEBUG("[serialDataSend]");
}