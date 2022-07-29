#include "agents/state/idleState.h"
#include "agents/state/waitState.h"
#include "agents/state/goToState.h"
#include "agents/state/conflictState.h"

using namespace agents;

IdleState::IdleState(Entity* pEntity)
{
    entity_ = pEntity;
    ROS_INFO("%s is in Idle state.", this->entity_->getID().c_str());
}

void IdleState::goTo(Pose pose, SocialSpace* sp)
{
    this->entity_->setState(new GoToState(this->entity_));
    this->entity_->goTo(pose, sp);
    delete this;
}

void IdleState::wait(float time)
{
    this->entity_->setState(new WaitState(this->entity_));
    this->entity_->wait(time);
    delete this;
}

void IdleState::wait(std::string topic, std::string event)
{
    this->entity_->setState(new WaitState(this->entity_));
    this->entity_->wait_event(topic, event);
    delete this;
}

bool IdleState::acceptAsyncTask()
{
    return true;
}