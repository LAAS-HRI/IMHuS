#ifndef WAIT_ACT
#define WAIT_ACT

#include "actions/navigateAction.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "agents/human.h"


//////////////////////////////////////////////////

////////////////// Wait Action /////////////////
namespace actions
{
    class WaitAction : public NavigateAction
    {
    public:
        // WaitAction();
        WaitAction(float duration);
        WaitAction(std::string topic, std::string event);
        ~WaitAction(){};

        virtual void execute();
        // float getDuration(){return duration_;};
        // void setDuration(float d){duration_=d;};

    private:
        // void eventCB(std_msgs::String);


        float duration_;
        ros::Time start_time_;
        ros::Duration dur_wait_;
        std::string topic_, event_;

        //ros
        ros::NodeHandle nh_;
        // ros::Subscriber listener_;
        
    };
}

#endif