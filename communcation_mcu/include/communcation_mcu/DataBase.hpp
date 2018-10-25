#pragma once 

#include "msgs/CmdVel.h"
#include "msgs/FeedBack.h"
#include <string>

/**
 * Robot's parameter
 **/
struct Parameter
{
    union{
        char buff[64];
        
        struct
        {   
            unsigned short wheel_diameter;       //轮子直径  mm
            unsigned short wheel_track;          //差分：轮距，
            unsigned short encoder_resolution;   //编码器分辨率
            unsigned char do_pid_interval;       //pid间隔 (ms)
            unsigned short kp;
            unsigned short ki;
            unsigned short kd;
            unsigned short ko;                   //pid参数比例
            unsigned short cmd_last_time;        //命令持久时间ms 超过该时间会自动停止运动
            unsigned short max_v_liner_x;
            unsigned short max_v_liner_y;
            unsigned short max_v_angular_z;
            unsigned char imu_type;
        };
    };
};

/**
 * Robot's algorithm parameter
 **/
struct PidData
{
    int input[4];  
    int output[4]; 
};

/**
 * The robot database
 **/
class DataBase
{
  public:
    static DataBase* get()
    {
        static DataBase instance;
        return &instance;
    }

    void load_parameter();

    void save_parameter();

  private:
    DataBase()
    {
        memset(&cmdvelData_, 0, sizeof(cmdvelData_));
        memset(&feedbackData_, 0 ,sizeof(feedbackData_));
    }

  public:
    struct Parameter  parameter_;
    struct PidData    pidData_;

    msgs::CmdVel cmdvelData_;
    msgs::FeedBack feedbackData_;

};