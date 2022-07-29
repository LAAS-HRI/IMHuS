#include "actions/waitAction.h"
#include "ros/ros.h"
#include "boost/foreach.hpp"
#include <thread>

using namespace actions;

WaitAction::WaitAction(float duration)
    : duration_(duration)
{
}

WaitAction::WaitAction(std::string topic, std::string event)
    : topic_(topic), event_(event)
{
}

void WaitAction::execute()
{
    if(! topic_.empty() && !event_.empty()) entity_->wait_event(topic_,event_);
    else entity_->wait(duration_);
    // std::thread thread{&agents::Entity::wait, entity_, duration_};
    // thread.join();
}
