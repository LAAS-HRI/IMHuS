#include "scenario/step.h"
#include "thread"
#include <vector>

using namespace scenario;

Step::Step(std::string _id)
{
    id = _id;
}

void Step::add(agents::Entity *entity, actions::CompoundTask *task)
{
    p_entity.push_back(entity);
    p_task.push_back(task);
}

void Step::execute()
{
    std::vector<std::thread*> threads;
    ROS_WARN("Starting step : %s", id.c_str());
    for(int i=0; i<this->p_task.size(); i++)
    {              
        threads.push_back(new std::thread(&actions::CompoundTask::execute, p_task[i], p_entity[i]));
    }
    for(auto &thread: threads)
    {
        thread->join();
    }
    for(auto &thread: threads) 
    {
        delete thread; //not sure its needed
    }
    ROS_INFO("Step done.");
}