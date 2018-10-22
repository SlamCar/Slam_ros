#include "Communcation.hpp"

Communcation::Communcation()
    : ConnectState_(false),ReciveFrequency_(50.0),SendFrequency_(5.0)
{
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    cmdVelSub_ = nh.subscribe<msgs::CmdVel>("cmdvel",
                                            10,
                                            boost::bind(&Communcation::updateCmd, this, _1));
    
    feedBackPub_ = nh.advertise<msgs::FeedBack>("feedback",100);

    setCommunicateType(CommunicateType::SERIAL);
}

Communcation::~Communcation()
{
    if (receiveThread_)
    {
        receiveThread_->interrupt();
        receiveThread_->join();
        receiveThread_ = nullptr;
    } 

    if(sendThread_) 
    {
        sendThread_->interrupt();
        sendThread_->join();
        sendThread_ = nullptr;
    }
}

bool Communcation::init()
{
    switch(Type_)
    {
        case CommunicateType::SERIAL: return serialInit();

        case CommunicateType::UDP:    return udpInit();
    }

    ROS_ERROR("Type of communication is none !!!");
    return false;
}

void Communcation::run()
{
    ROS_INFO_STREAM("MCU communcation started...");

    receiveThread_ = boost::make_shared<boost::thread>
                    (boost::bind(&Communcation::updateDataBase, this));
    
    sendThread_ = boost::make_shared<boost::thread>
                    (boost::bind(&Communcation::sendData, this));
}

void Communcation::updateDataBase()
{
    // SerialPackage RXbuffer;
    DataPack RXbuffer;

    ros::Rate loop(ReciveFrequency_);
    ROS_DEBUG("[updateDataBase...]");

    while (ros::ok())
    {
        if(!ConnectState_) 
        {
            ROS_ERROR("Connection is broken");
            return ;
        }

        if(serial_.isfree())
        {
            serial_.read(RXbuffer.data(), (  HEADER_BYTESIZE 
                                            + BODY_MAX_BYTESIZE
                                            + CRC_BYTESIZE ));

            ROS_DEBUG("dataId:[%x]",RXbuffer.dataId());
        }

        if(!RXbuffer.checkCrc())
        {
            ROS_WARN("data check error!");
            return;
        }

        switch(RXbuffer.dataId())
        {
            case STM32_FEED_BACK: 
                updateFeeback(RXbuffer.body());
                break;

            default:  
                ROS_WARN("no this dataId!!! ErrorId[%x]",RXbuffer.dataId());
                break;
        }
    

        if (loop.cycleTime() > ros::Duration(1.0 / ReciveFrequency_))
        {
            ROS_WARN("ReciveFrequency_ loop missed its desired rate of %.2fHz... the heartbeat actually took %.2fHz",
                        ReciveFrequency_, 1/loop.cycleTime().toSec());
        }

        // sleep to make sure the update frequency
        loop.sleep();
	    ros::spinOnce();
    }
}

void Communcation::sendData()
{
    ros::Rate loop(SendFrequency_);
    ROS_DEBUG("[sendData...]");

    while (ros::ok())
    {
        if(!ConnectState_) 
        {
            ROS_ERROR("Connection is broken");
            return ;
        }

        sendFeeback();
        sendCmd();

        if (loop.cycleTime() > ros::Duration(1.0 / SendFrequency_))
        {
            ROS_WARN("SendFrequency_ loop missed its desired rate of %.2fHz... the heartbeat actually took %.2fHz",
                        SendFrequency_, 1/loop.cycleTime().toSec());
        }

        // sleep to make sure the control frequency
        loop.sleep();
	    ros::spinOnce();
    }
    
}

bool Communcation::serialInit()
{
    /**
     * error check
     **/
    if ( CommunicateType::SERIAL != Type_ )
    {
        ROS_ERROR("Type of communication is not serial !!!");
    }
    
    ROS_DEBUG("CommunicateType: [Serial mode].");
    
    if(serial_.connect())
    {
        ROS_INFO("connected to main board"); 
    }
    else
    {
        ROS_ERROR("can't connect to main board!!! ");
        return false;
    }

    ros::Duration(2).sleep(); //wait for device
    ROS_INFO("end sleep"); 

    ConnectState_ = true;

    return true;
}

bool Communcation::udpInit()
{
    /**
     * TODO
     **/
}

void Communcation::sendCmd()
{
    DataBase* db = DataBase::get();

    DataPack cmdmsg(CMD_IPC_COMMOND);
    cmdmsg.setLen(sizeof(db->cmdvelData_));
    cmdmsg.setBody(reinterpret_cast<uint8_t *>(&db->cmdvelData_), sizeof(db->cmdvelData_));
    cmdmsg.generateCrc();

    serial_.write((uint8_t *)&cmdmsg, HEADER_BYTESIZE 
                                      + BODY_MAX_BYTESIZE
                                      + CRC_BYTESIZE); 
}

void Communcation::sendFeeback()
{
    DataBase* db = DataBase::get();

    msgs::FeedBack feedback;
    feedback.Velocity = db->feedbackData_.Velocity;
    feedback.Angle = db->feedbackData_.Angle;
    
    feedBackPub_.publish(feedback);  
}

void Communcation::updateCmd(const msgs::CmdVel::ConstPtr &cmdVel)
{
    ROS_DEBUG("[updateCmd]");

    DataBase* db = DataBase::get();
    
    db->cmdvelData_.driverVelocity = cmdVel->driverVelocity;
    db->cmdvelData_.steeringAngle = cmdVel->steeringAngle;
    ROS_DEBUG("update Cmd msg: Velocity = %f , Angle = %f",
                db->cmdvelData_.driverVelocity, db->cmdvelData_.steeringAngle);
 
}

void Communcation::updateFeeback(uint8_t* data)
{
    ROS_DEBUG("[updateFeeback]");

    DataBase* db = DataBase::get();
    
    memcpy(&db->feedbackData_, data, sizeof(db->feedbackData_));  
    ROS_DEBUG("update feedback msg: Velocity = %f , Angle = %f",
                db->feedbackData_.Velocity, db->feedbackData_.Angle);
}


