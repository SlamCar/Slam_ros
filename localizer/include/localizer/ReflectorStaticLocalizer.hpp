#pragma once
#include "ReflectorMap.hpp"
#include "Landmark.hpp"
#include "LandmarkReflectorMatcher.hpp"
#include "Geometry.hpp"
#include "SystemModel.hpp"
#include <ros/ros.h>
#include <vector>
#include <map>
#include <string>
#include <cmath>


// Two points and distance
using LineSegment = std::map<double, std::pair<geometry::Point, geometry::Point>>;

/**
 *  \brief [static reflector localizer algorithm]
 */
class ReflectorStaticLocalizer
{
  public:

    ReflectorStaticLocalizer(): distTreshold_(0.1)
    {
      F_.clear();
      P_.clear();
    }
    

    geometry::Pose getPosition(Landmarks landmarks, ReflectorMap reflectorMap);

    /**
     *  \brief Match without outliers
     *  \brief Input    M:[landmarks]  N:[reflectors]
     *  \brief Output   Pose(x,y,theta)
     */
    geometry::Pose GetPosition(const geometry::PointSet &M, const geometry::PointSet &N);

    void matchRemoveOutliers();
    bool setGlobalDistanceMap(const geometry::PointSet &N);

  private:
    /**
     *  Find the two points of the maximum distance 
     */
    LineSegment maxdist(const geometry::PointSet &M);


    /**
     *  [Match Characteristic data in the reflectors] 
     *  Input  Z:[Characteristic data]  N:[reflectors]
     *  Output F:[Matched reflectors] 
     */
    LineSegment f(const LineSegment &Z, const geometry::PointSet &N);

    /**
     *  
     *  return P: { (x1,y1),(x2,y2) ... }
     */
    geometry::PointSet select(const geometry::PointSet &M);


    geometry::Pose calcPose(const LineSegment &F, const LineSegment &N);
    
  private:
    // Characteristic data of landmarks
    LineSegment Z_;
    LineSegment F_;
    geometry::PointSet P_;

    LineSegment globalDistanceMap_;    
    double distTreshold_;
  
    void printfLineSegment(LineSegment value, std::string name)
    {
      uint id = 0;
      ROS_INFO_STREAM("LineSegment " << name << ":");
      for(auto itr = value.begin(); itr != value.end(); itr++)
      {
        id ++;
        ROS_INFO(" %d) dis:%f { (%f, %f),(%f, %f) }", id, itr->first,  
                      itr->second.first.x(), itr->second.first.y(),  
                      itr->second.second.x(), itr->second.second.y());
      }
      ROS_INFO("--------------------------------------------");
    }
};

