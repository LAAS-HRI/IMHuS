#include "map/socialSpace.h"
SocialSpace::SocialSpace(std::string _id, agents::Entity* _entities)
{
    setID(_id);
    entities = _entities;

    clearing = nh_.advertiseService("/"+id+"/clear", &SocialSpace::srvClear, this);
    last_check = ros::Time::now();
}

void SocialSpace::add(SocialPose p)
{
    this->social_poses.push_back(p);
}

Pose SocialSpace::requestPose()
{
    std::lock_guard<std::mutex> lg(some_mutex);
    if(last_check+ros::Duration(60)<ros::Time::now())
    {
        this->Clear();
        last_check = ros::Time::now();
    }
    this->checkIfFree();
    return this->assignPose();
}

void SocialSpace::checkIfFree()
{
    float x,y,d;
    for(SocialPose& p : this->social_poses)
    {
        p.free = true;
        for(auto agent : entities->getMembers())
        {
            agent->spinOnce();
            x = agent->getPose().getX();
            y = agent->getPose().getY();
            d = sqrt(pow(x-p.getX(), 2)+pow(y-p.getY(), 2));
            if(d < 2*agent->getRadius() || p.assigned) 
            {
                p.free = false;
                break;
            }
        }
    }
}

Pose SocialSpace::assignPose()
{
    for(SocialPose& p : this->social_poses)
    {
        if(p.free && !p.assigned)
        {
            p.free = false;
            p.assigned = true;
            
            Pose pose; //static cast?
            pose.setXYTheta(p.getX(), p.getY(), p.getTheta());
            return pose;
        }
    }
    ROS_WARN("%s is full, cannot assigned new poses !", id.c_str());
    SocialPose failure;
    return failure;
    //what todo if no pose is available ?
    // ros::Duration(5).sleep();
    // this->assignPose();
}


bool SocialSpace::srvClear(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    this->Clear();
    return true;
}

void SocialSpace::Clear()
{
    for(SocialPose& p : this->social_poses)
    {
        p.assigned = false;
    }
    this->checkIfFree();
    // ROS_INFO("Clearing assigned poses in socialspace %s",id.c_str());
}