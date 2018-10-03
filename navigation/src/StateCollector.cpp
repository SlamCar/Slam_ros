#include "StateCollector.hpp"


StateCollector::StateCollector()
{
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
 
    feedbackSub_ = nh.subscribe<msgs::Feedback>("Feedback", 5,
                                                      boost::bind(&StateCollector::feedbackCallback, this, _1));
    //CarPosePub_.advertise<msgs::CarPose>("CarPose",2);
}

void StateCollector::feedbackCallback(const msgs::Feedback::ConstPtr &feedback)
{
    state_ = WorkState::NORMOL;
    
}
