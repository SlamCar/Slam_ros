#include "ReflectorStaticLocalizer.hpp"

geometry::Pose ReflectorStaticLocalizer::getPosition(Landmarks landmarks, ReflectorMap reflectorMap)
{

    LandmarkReflectorMatcher lm;

    if(lm.matchRemoveOutliers(landmarks.toPointSet(), reflectorMap.toPointSet()))
    {
        // ROS_DEBUG("yes yes yes yes");
    }
    else
    {
        // ROS_DEBUG("no no no no");
    }

    if (!lm.poseList().empty() && 1 == lm.poseList().size())
    {
        double x = lm.poseList().front().x();
        double y = lm.poseList().front().y();
        double theta = lm.poseList().front().theta();
        ROS_DEBUG("static localizer Position: x: %f, y: %f, angle: %f", x, y, theta);
        
        return Pose(x,y,theta);
    }

}

geometry::Pose ReflectorStaticLocalizer::GetPosition(const geometry::PointSet &M, const geometry::PointSet &N)
{
    Z_ = maxdist(M);

    printfLineSegment(Z_, "Z");

    F_ = f(Z_, N);

    printfLineSegment(F_, "F");

    if(1 == F_.size())
    {
        return calcPose(Z_, F_);
    }
    else
    {
        ROS_ERROR("no no no");
        do
        {
            // n*(n-1) / 2 points of n lineSegment
            if(Z_.size() == M.size()*(M.size()-1) / 2)
            {
                ROS_ERROR("localizer faild");
                return;
            }

            P_ = select(M);
            Z_.append(P_);
            F_ = f(Z_,N);
        }
        while(F_.size() = 1);
    }

}

LineSegment ReflectorStaticLocalizer::maxdist(const geometry::PointSet &M)
{
    LineSegment z;
    double maxDist = 0.0;
    auto p1 = M.begin();
    auto p2 = M.begin();

    for(auto itr = M.begin(); itr != M.end(); itr++)
    {
        auto temp = itr;
        for(auto itr2 = ++temp ; itr2 != M.end(); itr2++)
        {
            double dis = std::hypot(itr->x()-itr2->x(), itr->y() - itr2->y());
            if(dis > maxDist)
            {
                maxDist = dis;
                p1 = itr;
                p2 = itr2;
            }
        }       
    }
    z[maxDist] = std::make_pair(*p1, *p2);

# if 0
    printfLineSegment(z, "z");
# endif

    return z;
}

LineSegment ReflectorStaticLocalizer::f(const LineSegment &Z, const geometry::PointSet &N)
{
    LineSegment f;

    for(auto itr = Z.begin(); itr != Z.end(); itr++)
    {
        for(auto MapItr = globalDistanceMap_.begin(); MapItr != globalDistanceMap_.end(); MapItr++)
        {
            if(std::fabs(itr->first - MapItr->first) < distTreshold_)
            {
                f[MapItr->first] = MapItr->second;
            }
        }
    
    }

    return f;    
}

geometry::Pose ReflectorStaticLocalizer::calcPose(const LineSegment &Z, const LineSegment &F)
{
    Eigen::MatrixXd A(4*Z.size(), 4);
    Eigen::MatrixXd b(4*Z.size(), 1);

    // set Matrix A
    auto zItr = Z.begin();
    A(0, 0) = zItr->second.first.x();
    A(0, 1) = -zItr->second.first.y();
    A(1, 0) = zItr->second.first.y();
    A(1, 1) = zItr->second.first.x();
    A(2, 0) = zItr->second.second.x();
    A(2, 1) = -zItr->second.second.y();
    A(3, 0) = zItr->second.second.y();
    A(3, 1) = zItr->second.second.x();

    for(uint i = 0; i < 2; i++)
    {
        A(i*2,2) = 1;
        A(i*2,3) = 0;
        A(i*2+1, 2) = 0;
        A(i*2+1, 3) = 1;
    }
    // std::cout << "A:" << std::endl;
    // std::cout << A << std::endl;

    auto fItr = F.begin();
    b(0, 0) = fItr->second.first.x();
    b(1, 0) = fItr->second.first.y();
    b(2,0) = fItr->second.second.x();
    b(3,0) = fItr->second.second.y();
    // std::cout << "b:" << std::endl;
    // std::cout << b << std::endl;
    // calculate the matrix of pose
    Eigen::MatrixXd H = (A.transpose()*A).inverse()*A.transpose()*b;

    ROS_DEBUG("x: %f, y: %f", H(2), H(3));

    return Pose(H(2), H(3), atan2(H(1), H(0)));
}

bool ReflectorStaticLocalizer::setGlobalDistanceMap(const geometry::PointSet &N)
{
    if(N.size() <= 1 )
    {
        ROS_ERROR("reflector size is not enough");
        return false;
    }

    for(auto itr = N.begin(); itr != N.end(); itr++)
    {
        auto temp = itr;
        for(auto itr2 = ++temp ; itr2 != N.end(); itr2++)
        {
            double dis = std::hypot(itr->x()-itr2->x(), itr->y() - itr2->y());
            globalDistanceMap_[dis] = std::make_pair(*itr, *itr2);
        }       
    }


# if 0
    printfLineSegment(globalDistanceMap_, "globalDistanceMap");
#endif

    return true;
}
