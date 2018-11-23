#include "Communcation.hpp"

Communcation::Communcation()
    : ConnectState_(false),ReciveFrequency_(50.0),SendFrequency_(50.0)
{
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    cmdVelSub_ = nh.subscribe<msgs::CmdVel>("cmdvel",
                                            10,
                                            boost::bind(&Communcation::updateCmd, this, _1));
    
    feedBackPub_ = nh.advertise<msgs::FeedBack>("feedback",100);

    updater_[STM32_FEED_BACK] = boost::bind(&Communcation::updateFeeback, this, _1);
    updater_[STM32_HEART_BEAT] = boost::bind(&Communcation::updateHeartbeat, this, _1);
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
    setCommunicateType(CommunicateType::SERIAL);

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
            auto dataSize = serial_.read(RXbuffer.data());
            ROS_DEBUG("dataId:[%x]",RXbuffer.dataId());
            
            #if 0
            ROS_DEBUG("receicve data size: %d",static_cast<int>(dataSize));
            RXbuffer.body();
            #endif

            try
            {
                if(!RXbuffer.checkCrc())
                {
                    ROS_WARN("data check error!");
                }
                else if(0x0000 == RXbuffer.dataId())
                {
                    ROS_WARN("empty data");
                }
                else 
                {
                    auto update = updater_.at(RXbuffer.dataId());
                    update(RXbuffer);
                }
            }
            catch (const std::out_of_range &e)
            {
                ROS_ERROR("No such command handler: %x", RXbuffer.dataId());
            }
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

    DataPack cmdMsg(CMD_IPC_COMMOND);
    cmdMsg.setLen(sizeof(db->cmdvelData_));
    cmdMsg.setBody(reinterpret_cast<uint8_t *>(&db->cmdvelData_), sizeof(db->cmdvelData_));
    cmdMsg.generateCrc();

    serial_.write((uint8_t *)&cmdMsg, cmdMsg.availableSize()); 
   
    #if 1
    ROS_DEBUG("send cmdMsg size : %d", static_cast<int>(cmdMsg.availableSize()));
    #endif
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

void Communcation::updateFeeback(DataPack msg)
{
    ROS_DEBUG("[updateFeeback]");
    
    DataBase* db = DataBase::get();
    
    memcpy(&db->feedbackData_, msg.body(), sizeof(db->feedbackData_));  
    ROS_DEBUG("update feedback msg: Velocity = %f , Angle = %f",
                db->feedbackData_.Velocity, db->feedbackData_.Angle);
}

void Communcation::updateHeartbeat(DataPack msg)
{
    ROS_DEBUG("[updateHeartbeat]");
}


