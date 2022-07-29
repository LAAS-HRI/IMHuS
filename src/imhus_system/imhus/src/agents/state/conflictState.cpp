#include "agents/state/conflictState.h"
#include "agents/state/waitState.h"
#include "agents/state/idleState.h"
#include "agents/state/goToState.h"

using namespace agents;

ConflictState::ConflictState(Entity* pEntity)
{
    entity_ = pEntity;
    ROS_WARN("%s is in Conflict state.", entity_->getID().c_str());
    init_ = ros::Time::now();
}

void ConflictState::goTo(Pose pose, SocialSpace* sp)
{
    Pose p=entity_->getPose(); //to refresh
    if(sp != nullptr)
    {
        ROS_INFO("%s is asking %s for a new pose.", entity_->getID().c_str(), sp->getID().c_str());
        Pose new_pose = sp->requestPose();
        //try to plan to this one
        entity_->setState(new GoToState(entity_));
        entity_->goTo(new_pose, sp);
        delete this;
    }
    else
    {
        //find another pose near first goal
        ROS_INFO("%s is in conflict so it will stop there.", entity_->getID().c_str());
        entity_->setState(new IdleState(entity_));
        delete this;
    }
}

void ConflictState::wait(float time)
{
    // ROS_INFO("%s starts waiting for %f seconds...",entity_->getID().c_str(), time);
    // ros::Duration(time).sleep();
    // entity_->setState(new IdleState(entity_));
    // delete this;
}

void ConflictState::wait(std::string topic, std::string event)
{
    
    // this->entity_->setFreeSub(topic);
    // ROS_INFO("%s starts waiting for event %s on topic %s...",entity_->getID().c_str(), event.c_str(), topic.c_str());

    // while(this->entity_->event != event)
    // {
    //     ros::Duration(0.1).sleep();
    //     this->entity_->spinOnce();
    // }
    // this->entity_->setFreeSub("");
    // ROS_INFO("Event seen !");
    // entity_->setState(new IdleState(entity_));
    // delete this;
}