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
    ser_.connect();
    return true;
}

void Communcation::cmdVelCallback(const msgs::CmdVel::ConstPtr &cmdVel)
{
    dataSend(cmdVel);
    // ROS_DEBUG("[CMD_VEL]");
}

void Communcation::dataSend(const msgs::CmdVel::ConstPtr &cmdVel)
{
    switch(sendType_)
    {
    case CommunicateType::SERIAL:
    {
        ROS_DEBUG_STREAM("CommunicateType: Serial mode.");
        serialDataSend(cmdVel);
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

void Communcation::serialDataSend(const msgs::CmdVel::ConstPtr &cmdVel)
{
    TaskRequest task;
    task.driverVelocity = cmdVel->driverVelocity;
    task.steeringAngle = cmdVel->steeringAngle;

    SerialPack pack(CmdType::TASK_REQ);
    pack.setLen(20);
    pack.setBody(reinterpret_cast<uint8_t *>(&task), pack.len());
    pack.generateCrc();

    ser_.write(reinterpret_cast<uint8_t *>(&pack), sizeof(pack));

    std::memset(reinterpret_cast<uint8_t *>(&pack), 0, sizeof(pack));
    ROS_DEBUG("[serialDataSend]");
}
