#include "scenario/scenarios.h"

using namespace scenario;

Scenario* Scenarios::get(std::string id)
{
    for(auto s : scenarios_)
    {
        if(s->id == id) return s;
    }
    ROS_ERROR("scenario::get() was not able to find scenario: %s. Check id.", id.c_str());
    return scenarios_[0];
}

void Scenarios::add(Scenario* new_s)
{
    if(!this->isAlreadyHere(new_s)) scenarios_.push_back(new_s);
}


bool Scenarios::isAlreadyHere(Scenario* sce)
{
    for(auto s : scenarios_)
    {
        if(s->id == sce->id) return true;
    }
    return false;
}