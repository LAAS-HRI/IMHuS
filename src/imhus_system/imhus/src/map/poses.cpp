#include "map/poses.h"

Pose Poses::get(std::string id)
{
    for(auto pose : poses)
    {
        if(pose.getID() == id) return pose;
    }
    ROS_ERROR("poses::get() was not able to find pose: %s. Check id.", id.c_str());
    return poses[0];
}


bool Poses::isAlreadyHere(Pose pose)
{
    for(auto p : poses)
    {
        if(p.getID() == pose.getID()) 
        {
            ROS_WARN("Skipping adding pose %s because of redunduncy of ids.", pose.getID().c_str());
            return true;
        }
    }
    return false;
}

void Poses::add(Pose new_pose)
{
    if(!this->isAlreadyHere(new_pose)) poses.push_back(new_pose);
}