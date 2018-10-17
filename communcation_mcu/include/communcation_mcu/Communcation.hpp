#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/assign/list_of.hpp>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <map>
#include <boost/thread.hpp>

#include "msgs/CmdVel.h"
#include "msgs/FeedBack.h"
#include "SerialPack.hpp"
#include "UART_Interface.hpp"
#include "SerialPack.hpp"
#include "DataBase.hpp"

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

     ~Communcation();

    bool init();

    void run();

  private:

    Communcation();

    bool serialInit();
    bool udpInit();
    
    void updateDataBase();
    void sendData();
    
    bool ConnectState_;
    double ReciveFrequency_;
    double SendFrequency_;
    CommunicateType Type_;                      
    McuSerial serial_;
    
    boost::shared_ptr<boost::thread> sendThread_;
    boost::shared_ptr<boost::thread> receiveThread_;

    ros::Subscriber cmdVelSub_;
    ros::Publisher feedBackPub_;                 
    ros::Publisher carParamPub_;                // some Param of the car

    // std::map<uint16_t, std::function<void(DataPack)>> receive_;

    msgs::FeedBack feedBackMsg_;
    
    inline void setCommunicateType(CommunicateType type) {Type_ = type;}
    /**
     * update  database 
     **/
    void updateCmd(const msgs::CmdVel::ConstPtr &cmdVel);
    void updateFeeback(); 

    /**
     * send  data 
     **/
    void sendCmd();
    void sendFeeback();
};