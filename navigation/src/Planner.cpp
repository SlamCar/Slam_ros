#pragma once
#include "cotek_msgs/CmdVel.h"

#include "ros/ros.h"
#include <map>

class Planner
{
  public:
    Planner();
    inline void enableFreeController() { enableFreeController_ = true; }
    inline void enableMagController() { enableMagController_ = true; }
    inline void disableFreeController() { enableFreeController_ = false; }
    inline void disableMagController() { enableMagController_ = false; }
    void update(const TaskPtr tm, const StatePtr sm);
    cotek_msgs::CmdVel plan(const TaskPtr tm, const StatePtr sm);

  private:
    bool setControllerByNaviType(NaviType type);

    inline bool isHybrid() { return enableFreeController_ && enableMagController_; }

    bool enableFreeController_;
    bool enableMagController_;
    FreeController freeCtrl_;
    MagneticController magCtrl_;
    AbstractController *controller_;
    NaviType naviType_;

    std::map<NaviType, std::map<NaviType, std::function<void(const Site &)>>> transferHandler_;

    ros::ServiceClient localizerSrv_;
    ros::ServiceClient switchSectionAndPoseSrv_;
};