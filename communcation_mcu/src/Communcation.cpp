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
    
    return true;
}

void Communcation::cmdVelCallback(const msgs::CmdVel::ConstPtr &cmdVel)
{

    // send2Slave(IPC_RSP, reinterpret_cast<uint8_t *>(&req), sizeof(IpcRequest));
    ROS_DEBUG("[CMD_VEL]");
}