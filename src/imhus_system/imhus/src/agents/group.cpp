#include "agents/group.h"

using namespace agents;

Group::Group()
{
    ROS_ERROR("Group constructed without id !");
    id_ = "Nameless";
    init_time_ = ros::Time::now();
    // this->state_ = new IdleState(this);
}

Group::Group(std::string id)
{
    id_ = id;
    init_time_ = ros::Time::now();
    // this->state_ = new IdleState(this);
}


void Group::add(Entity* ptr)
{
    if(!this->isAlreadyHere(ptr->getID())) members_.push_back(ptr);
}

Entity* Group::getEntityPtr(std::string id)
{
    for(auto member_ptr : members_)
    {
        if(member_ptr->getID() == id) return member_ptr;
    }

    //the if is to avoid the msg when the group is being created
    //at initial time the agentsa are not in the group but it is normal
    if(ros::Time::now()>init_time_+ros::Duration(10)) ROS_ERROR("Entity wasn't able to find the ptr of agent from the id %s. May make the script crash. \n (Ignore if this message shows up when launching the system.)", id.c_str());
    return nullptr;
}

std::vector<Entity*>::iterator Group::getMembersIt()
{
    std::vector<Entity*>::iterator it = members_.begin();
    return members_.begin();
}

std::vector<Entity*> Group::getMembers()
{
    return members_;
}

bool Group::isAlreadyHere(std::string id)
{
    return this->getEntityPtr(id)!=nullptr;
}

void Group::wait(float duration)
{
    std::vector<std::thread> threads;

    for(auto agent : members_)
    {
        threads.push_back(std::thread(&agents::Entity::wait, agent, duration));
    }
    for(auto &thread: threads) thread.join();
}

void Group::wait_event(std::string topic, std::string event)
{
    std::vector<std::thread> threads;

    for(auto agent : members_)
    {
        threads.push_back(std::thread(&agents::Entity::wait_event, agent, topic, event));
    }
    for(auto &thread: threads) thread.join();

}

void Group::goTo(Pose pose, SocialSpace* sp)
{ 
    // this->state_->goTo(p);
    // std::vector<std::thread> threads;
    // for(auto member : this->members_)
    // {
    //     threads.push_back(std::thread(&agents::Entity::state_::goTo, member, pose));
    // }
    // for(auto &thread: threads) thread.join();
}

void Group::poseCB(nav_msgs::Odometry msg)
{
    //nothing to do
}
void Group::sendPlan(nav_msgs::Path p)
{
    ROS_ERROR("A group does not send plans !");
}

void Group::update()
{
    
}