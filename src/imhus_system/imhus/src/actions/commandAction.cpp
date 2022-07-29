#include "actions/commandAction.h"

#include <iostream>
#include "ros/ros.h"

using namespace actions;

CommandAction::CommandAction()
{}

void CommandAction::set(agents::Entity* new_)
{
    entity_ = new_;
}

