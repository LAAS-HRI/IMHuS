#include "scenario/scenario.h"

using namespace scenario;

void Scenario::execute()
{
    ROS_WARN("\n\n \t Starting scenario : %s. \n", id.c_str());
    stopping_request=false;
    std::vector<std::thread *> async_threads;
    for (auto step : this->steps.getSteps())
    {
        if(!stopping_request)
        {
            //special case of async actions in this scenario
            if (step->id == "AsyncActions" || step->id == "AsyncAction" || step->id == "async_actions" || step->id == "async_action")
            {
                int k=0;
                for (int i = 0; i < step->p_task.size(); i++)
                {
                    k++;
                    async_threads.push_back(new std::thread(&scenario::Step::execute, step));
                }
                ROS_WARN("There are %d AsyncActions pending in this scenario.", k);
                for (auto &thread : async_threads)
                {
                    thread->detach();
                }
            }
            else
            {
                //normal execution of main scenario
                step->execute();
            }
        }
        else
        {
            ROS_WARN("The scenario has been terminated early on request.");
        }
    }
    //delete thread of async task, seems to be okay if we dont do it but its cleaner to do it
    for(auto &thread : async_threads)
    {
        thread->~thread();
        delete thread;
    } 
    ROS_WARN("\n\n \t Scenario done ! \n");
}