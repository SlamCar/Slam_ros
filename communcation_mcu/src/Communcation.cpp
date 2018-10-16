#include "Communcation.hpp"

typedef std::vector<uint8_t> Buffer;

Communcation::Communcation()
    : ConnectState_(false),CommuncateFrequency_(10.0)
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

void Communcation::updateDataBase()
{
    ros::Rate loop(CommuncateFrequency_);
    ROS_DEBUG("[Data Uptating...]");

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
    // if (need_update_speed_)
    // {
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
    
    // }
}

void Communcation::updateFeeback()
{
    SerialPackage feedbackmsg;
    
   // DataPack 

    DataBase* db = DataBase::get();
    
    if(serial_.isfree())
    {
        ROS_DEBUG("[updateFeedback]");
        serial_.read((uint8_t *)&feedbackmsg,(  HEADER_BYTESIZE 
                                              + sizeof(db->feedbackData_)
                                              + CRC_BYTESIZE ));
        //check


        ROS_INFO("moduleId:[0x%02x]",feedbackmsg.head_.moduleId);
        
        memcpy(&db->feedbackData_,&feedbackmsg.byData_,sizeof(db->feedbackData_)); 
        
        msgs::FeedBack feedback;
        feedback.Velocity = db->feedbackData_.Velocity;
        feedback.Angle = db->feedbackData_.Angle;
        ROS_DEBUG("feedback msg: Velocity = %f , Angle = %f",feedback.Velocity, feedback.Angle);
        feedBackPub_.publish(feedback);   
    }   
}


//bool Communcation::dataRight(SerialPackage msg,uint8_t checksum)
//{
    // static uint8_t flag = 0;
    // static uint8_t len = 0;

    // switch(flag)
    // {
    // /*--------------------head---------------------*/    
    //      /***moduleId***/
    //     case 0: 
    //         rxpack.clear(); 
    //         if (data == 0x03) 
    //         {
    //             rxpack.push_back(data);
    //             flag ++;
    //         }
    //         else flag = 0;
    //         break;
    //     case 1:
    //         if (data == 0x9c) 
    //         {
    //             rxpack.push_back(data);
    //             flag ++;
    //         }
    //         else 
    //         {
    //             flag = 0;
    //             rxpack.clear(); //头错误，就清空buffer
    //         }
    //     /***push dataId***/
    //     case 3: rxpack.push_back(data); flag ++;
    //         break;
    //     case 4: rxpack.push_back(data); flag ++;
    //         break;
    //     /***push dataLen***/
    //     case 5: rxpack.push_back(data); flag ++;
    //         break;
    //     /***push recv_len***/
    //     case 6: rxpack.push_back(data); flag ++;

    // /*--------------------data---------------------*/
    //     case 7: 
    //         if (rxpack.at(5) == len)   //相当于rxpack[5] == len 数据放完就
    //         {
    //             len = 0;
    //             flag ++;
    //         } 
    //         rxpack.push_back(data);
    //         len++;
    //         if(len > rxpack.at(5))
    //         {
    //             rxpack.clear();
    //             ROS_ERROR("Data overflow!!!");
    //             return false;
    //         }
    //         break;

    // /*--------------------check---------------------*/
    //     case 8: rxpack.push_back(data); flag = 53;
    //         break;

    // /*------------------receive ok!!!---------------------*/        
    //     case 53: return true;  
    // }
    // return false;
//}
