/** 
 * Copyright (c) 2018 old_wang Inc. All rights reserved. 
 */
#pragma once

class Communcation
{
public:

    static Communcation &getInstance()
    {
        static Communcation instance;
        return instance;
    }

    SwarmClient();

    virtual ~SwarmClient() {}

    bool start();

private:
    void cmdVelCallback(const msgs::CmdVel::ConstPtr &cmdVel);
    
    
    ros::Subscriber cmdVelSub_;
    ros::Publisher feedbackPub_;
};