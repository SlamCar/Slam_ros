#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/assign/list_of.hpp>
#include <string>
#include <vector>
#include <ros/ros.h>

#include "msgs/CmdVel.h"
#include "msgs/FeedBack.h"
#include "UART_Interface.hpp"
#include "SerialPack.hpp"
#include "SerialTransport.hpp"
#include "Transport.hpp"
#include "DataBase.hpp"
#include "IPC_Protocalframe.hpp"


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

    virtual ~Communcation() {}

    bool init();

    void updating();


  private:

    Communcation();
    bool serialInit();
    bool udpInit();
    
    bool ConnectState_;
    bool need_update_speed_;
    double CommuncateFrequency_;
    CommunicateType Type_;                      //Communcate of the car
    McuSerial serial_;
    // boost::shared_ptr<McuSerial> serial_;

    ros::Subscriber cmdVelSub_;
    ros::Publisher feedBackPub_;                 
    ros::Publisher carParamPub_;                // some Param of the car

    msgs::FeedBack feedBackMsg_;
 
    boost::shared_ptr<Transport> trans_;        // class for serial or udp
    boost::shared_ptr<Protocalframe> frame_;    // class for protocal_package
    
    inline void setCommunicateType(CommunicateType type) {Type_ = type;}
    
    /**
     * save data : external --> database  
     **/
    void cmdVelCallback(const msgs::CmdVel::ConstPtr &cmdVel);

    /**
     * update data :  database --> external
     **/
    void updateCmd();
    void updateFeeback(); //board --> database --> pub

    /**
     * data check
     **/
    bool dataRight(SerialPackage msg,uint8_t checksum);
    
    

};