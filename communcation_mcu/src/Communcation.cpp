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

    // receive_[STM32_FEED_BACK] = boost::bind(&Communcation::updateFeeback, this, _1);
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
    //ros::Rate loop(ReciveFrequency_);
    ROS_DEBUG("[updateDataBase...]");
    while (ros::ok())
    {
        if(!ConnectState_) 
        {
            ROS_ERROR("Connection is broken");
            return ;
        }
        #if 0
        if(serial_.isfree())
        {
        serial_.read((uint8_t *)&pack,(  HEADER_BYTESIZE 
                                        + sizeof(db->feedbackData_)
                                        + CRC_BYTESIZE ));
        switch(id)
        {
            case STM32_FEED_BACK: updateFeeback();

        }
        #endif
        updateFeeback();

        // if (loop.cycleTime() > ros::Duration(1.0 / ReciveFrequency_))
        // {
        //     ROS_WARN("ReciveFrequency_ loop missed its desired rate of %.2fHz... the heartbeat actually took %.2fHz",
        //                 ReciveFrequency_, 1/loop.cycleTime().toSec());
        // }

        // // sleep to make sure the update frequency
        // loop.sleep();
	    // ros::spinOnce();
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

void Communcation::sendFeeback()
{
    DataBase* db = DataBase::get();

    msgs::FeedBack feedback;
    feedback.Velocity = db->feedbackData_.Velocity;
    feedback.Angle = db->feedbackData_.Angle;
    //ROS_DEBUG("send feedback msg: Velocity = %f , Angle = %f",feedback.Velocity, feedback.Angle);
    feedBackPub_.publish(feedback);  
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

void Communcation::updateCmd(const msgs::CmdVel::ConstPtr &cmdVel)
{
    ROS_DEBUG("[updateCmd]");

    DataBase* db = DataBase::get();
    db->cmdvelData_.driverVelocity = cmdVel->driverVelocity;
    db->cmdvelData_.steeringAngle = cmdVel->steeringAngle;
    
    /**
     * Test
     * */
    #if 0
    db->cmdvelData_.driverVelocity = 1.2;
    db->cmdvelData_.steeringAngle = 3.4;
    #endif
    SerialPackage cmdmsg(CMD_IPC_COMMOND, (uint8_t *)&db->cmdvelData_, sizeof(db->cmdvelData_));
    serial_.write((uint8_t *)&cmdmsg,(sizeof(db->cmdvelData_)+7)); //15
    
}

void Communcation::updateFeeback()
{
    SerialPackage feedbackmsg;

    memset(&feedbackmsg, 0, sizeof(SerialPackage));

   // DataPack 
    DataBase* db = DataBase::get();
    
    if(serial_.isfree())
    {
        // serial_.read((uint8_t *)&feedbackmsg,(  HEADER_BYTESIZE 
        //                                       + sizeof(db->feedbackData_)
        //                                       + CRC_BYTESIZE ));
        serial_.read((uint8_t *)&feedbackmsg,(  HEADER_BYTESIZE 
                                              + BODY_MAX_BYTESIZE
                                              + CRC_BYTESIZE ));

        #if 0
        for(int t = 0; t < sizeof(db->feedbackData_); t++)
            ROS_INFO("[0x%02x]",feedbackmsg.byData_[t]);

        ROS_INFO("_______________________________________");
        #endif

        if(feedbackmsg.head_.moduleId != 0x039c)
            return;
        
        uint16_t crc = 0;
        uint8_t *buffer = (uint8_t *)&feedbackmsg;
        for(int i = 0; i < HEADER_BYTESIZE + feedbackmsg.head_.dataLen; i++)
            crc += buffer[i];       
    
        ROS_WARN_COND(crc != feedbackmsg.check_ ,
                     "feedbackmsg.check:[%x], crc[%x]",
                     feedbackmsg.check_, crc);

        // ROS_INFO("moduleId : %x",feedbackmsg.head_.moduleId);
        //ROS_INFO("data crc : %x",feedbackmsg.check_);

        if(crc == feedbackmsg.check_)
        {
            memcpy(&db->feedbackData_,&feedbackmsg.byData_,sizeof(db->feedbackData_));  
            ROS_DEBUG("update feedback msg: Velocity = %f , Angle = %f",db->feedbackData_.Velocity, db->feedbackData_.Angle);
        }
        else
        {
            ROS_WARN("data error");
        }

    }   
}


