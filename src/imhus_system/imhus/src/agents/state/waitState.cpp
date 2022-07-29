#include "agents/state/waitState.h"
#include "agents/state/idleState.h"
#include "agents/state/goToState.h"
#include "agents/state/conflictState.h"

using namespace agents;

WaitState::WaitState(Entity* pEntity)
{
    entity_ = pEntity;
    ROS_INFO("%s is in Waiting state.", entity_->getID().c_str());
}

void WaitState::goTo(Pose pose, SocialSpace*)
{
    //TODO
    ROS_ERROR("Asks me to go but Im waiting, what should I do ?");
}

void WaitState::wait(float time)
{
    ROS_INFO("%s starts waiting for %f seconds...",entity_->getID().c_str(), time);
    ros::Duration(time).sleep();
    entity_->setState(new IdleState(entity_));
    delete this;
}

void WaitState::wait(std::string topic, std::string event)
{
    
    this->entity_->setFreeSub(topic);
    ROS_INFO("%s starts waiting for event %s on topic %s...",entity_->getID().c_str(), event.c_str(), topic.c_str());

    while(this->entity_->event != event)
    {
        ros::Duration(0.1).sleep();
        this->entity_->spinOnce();
    }
    this->entity_->setFreeSub("");
    ROS_INFO("Event seen !");
    entity_->setState(new IdleState(entity_));
    delete this;
}