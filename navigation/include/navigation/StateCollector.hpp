#pragma once 

#include <ros/ros.h>

#include "Pose.hpp"
#include "msgs/Feedback.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Int32.h"
#include <std_msgs/Float64.h>

typedef struct {
    int status;//小车状态，0表示未初始化，1表示正常，-1表示error
    float power;//电源电压【 】v
    float theta;//方位角，【0 360）°
    int encoder_ppr;//车轮1转对应的编码器个数
    int encoder_delta_r;//右轮编码器增量， 个为单位
    int encoder_delta_l;//左轮编码器增量， 个为单位
    int encoder_delta_car;//两车轮中心位移，个为单位
    int omga_r;//右轮转速 个每秒
    int omga_l;//左轮转速 个每秒
    float distance1;//第一个超声模块距离值 单位cm
    float distance2;//第二个超声模块距离值 单位cm
    float distance3;//第三个超声模块距离值 单位cm
    float distance4;//第四个超声模块距离值 单位cm
    float IMU[9];//mpu9250 9轴数据
    unsigned int time_stamp;//时间戳
}UPLOAD_STATUS;

enum class WorkState : uint32_t
{
    NORMOL = 0,
    DEBUG = 1,
    ERROR =2
};

class StateCollector
{
  public:
    StateCollector();
    ~StateCollector();

    void update();
    
    void setCarTwist();
    void setCarOdom();
    void setCarPower();

    inline Pose getCarPose() { return CarPose_; }
    inline WorkState getState() { return state_; }
    nav_msgs::Odometry getOdom();
    inline std_msgs::Float64 getPower() { return CarPower_; }

    
  private:
    void feedbackCallback(const msgs::Feedback::ConstPtr &feedback);

    WorkState state_;
    Pose CarPose_;
    //geometry_msgs::Pose2D CarPose_;
    geometry_msgs::Twist CarTwist_;
    nav_msgs::Odometry CarOdom_;
    std_msgs::Float64 CarPower_;
    
    ros::Subscriber feedbackSub_;
    ros::Publisher CarPosePub_;
    ros::Publisher CarTwistPub_;
    // ros::Publisher 
    // ros::Publisher 

};