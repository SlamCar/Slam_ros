#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/assign/list_of.hpp>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <map>

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

    virtual ~Communcation() {}

    bool init();

    void updateDataBase();

    void sendData();


  private:

    Communcation();
    bool serialInit();
    bool udpInit();
    
    bool ConnectState_;
    double CommuncateFrequency_;
    CommunicateType Type_;                      
    McuSerial serial_;

    ros::Subscriber cmdVelSub_;
    ros::Publisher feedBackPub_;                 
    ros::Publisher carParamPub_;                // some Param of the car

    // std::map<uint16_t, std::function<void(DataPack)>> receive_;

    msgs::FeedBack feedBackMsg_;
    
    inline void setCommunicateType(CommunicateType type) {Type_ = type;}

    /**
     * update data 
     **/
    void updateCmd(const msgs::CmdVel::ConstPtr &cmdVel);
    void updateFeeback(); //board --> database --> pub

};