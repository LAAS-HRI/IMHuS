#include "actions/publishAction.h"

using namespace actions;

PublishAction::PublishAction()
{}

PublishAction::PublishAction(std::string topic, std::string msg)
:topic_(topic), msg_(msg)
{

}

void PublishAction::execute()
{
    entity_->publish(topic_, msg_);
}