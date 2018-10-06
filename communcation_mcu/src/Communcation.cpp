#include "Communcation.hpp"

typedef std::vector<uint8_t> Buffer;

Communcation::Communcation()
    : ConnectState_(false),CommuncateFrequency_(10.0), trans_(nullptr), frame_(nullptr)
{
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    need_update_speed_ = false;

    cmdVelSub_ = nh.subscribe<msgs::CmdVel>("cmdvel",
                                            10,
                                            boost::bind(&Communcation::cmdVelCallback, this, _1));
    
    feedBackPub_ = nh.advertise<msgs::FeedBack>("feedback",100);

    setCommunicateType(CommunicateType::SERIAL);
    

    // trans_ = Type_ == CommunicateType::SERIAL ? 
    //                   boost::make_shared<Serial_transport>("/dev/ttyUSB0", 115200) : nullptr;
    
    // frame_ = boost::make_shared<Ipc_dataframe>(trans_.get());
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

void Communcation::updating()
{
    ros::Rate loop(CommuncateFrequency_);
    ROS_DEBUG("[Uptating...]");
    updateCmd();

    while (ros::ok())
    {
        if(!ConnectState_) 
        {
            ROS_ERROR("Connection is broken");
            return ;
        }

       // updateCmd();

        updateFeeback();
		

        loop.sleep();
	    ros::spinOnce();
    }
}

/********************Private member function********************/

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

    // if (trans_->init())
    // {
    //     ROS_INFO("connected to main board");
    // } 
    // else
    // {
    //     ROS_ERROR("can't connect to main board!!! ");
    //     return false;
    // }

    ros::Duration(2).sleep(); //wait for device
    ROS_INFO("end sleep"); 

    ConnectState_ = true;
    // frame_->init();
        // Buffer data ; 
        // data.push_back('1');
        // data.push_back('\r');
        // data.push_back('\n');
        // trans_->write(data);
        //frame_->interact(DEBUG_TEST_COMMOND);
    //frame_->interact(CMD_IPC_COMMOND);

    return true;
}

bool Communcation::udpInit()
{
    /**
     * TODO
     **/
}


void Communcation::cmdVelCallback(const msgs::CmdVel::ConstPtr &cmdVel)
{
    DataBase* db = DataBase::get();

    db->cmdvelData_.driverVelocity = cmdVel->driverVelocity;
    db->cmdvelData_.steeringAngle = cmdVel->steeringAngle;

    need_update_speed_ = true;
}

void Communcation::updateCmd()
{
    // if (need_update_speed_)
    // {
        ROS_DEBUG("[updateCmd]");
        DataBase* db = DataBase::get();

        db->cmdvelData_.driverVelocity = 0.0;
        db->cmdvelData_.steeringAngle = 0.0;
    
        SerialPackage cmdmsg(CMD_IPC_COMMOND, (uint8_t *)&db->cmdvelData_, sizeof(db->cmdvelData_));
        serial_.write((uint8_t *)&cmdmsg,(sizeof(db->cmdvelData_)+7)); //15
    
    // }
}

void Communcation::updateFeeback()
{
    SerialPackage feedmsg;
    DataBase* db = DataBase::get();
    
    if(serial_.isfree())
    {
        ROS_DEBUG("[updateFeedback]");
        serial_.read((uint8_t *)&feedmsg,(sizeof(db->feedbackData_)+7));

        ROS_INFO("moduleId:[0x%02x]",feedmsg.head_.moduleId);
        
        memcpy(&db->feedbackData_,&feedmsg.byData_,sizeof(db->feedbackData_)); 
        
        msgs::FeedBack feedback;
        feedback.Velocity = db->feedbackData_.Velocity;
        feedback.Angle = db->feedbackData_.Angle;
        feedBackPub_.publish(feedback);   
    }   
}

bool Communcation::dataRight(SerialPackage msg,uint8_t checksum)
{
    

}
