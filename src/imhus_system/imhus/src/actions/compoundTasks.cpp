#include "actions/compoundTasks.h"

using namespace actions;

CompoundTasks::CompoundTasks()
{
}

void CompoundTasks::add(CompoundTask* task_)
{
    this->tasks_.push_back(task_);
}


CompoundTask* CompoundTasks::getTask(std::string id)
{
    for(auto task : tasks_)
    {
        if(task->getID() == id) return task;
    }
    ROS_ERROR("The search for task %s failed. Giving task 0 as a result.", id.c_str());
    return tasks_[0];
}