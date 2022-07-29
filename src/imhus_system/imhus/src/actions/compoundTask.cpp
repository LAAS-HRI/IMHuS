#include "actions/compoundTask.h"

using namespace actions;

CompoundTask::CompoundTask(std::string id)
{
    id_ = id;
}

void CompoundTask::add(CommandAction* action_)
{
    this->actions_.push_back(action_);
}

void CompoundTask::execute(agents::Entity* ent)
{
    for(auto action : actions_)
    {
        action->set(ent);
        action->execute();
    }
}
