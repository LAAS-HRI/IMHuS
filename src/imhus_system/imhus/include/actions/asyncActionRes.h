#ifndef ASYNC_ACTIONRES
#define ASYNC_ACTIONRES

#include "actions/commandAction.h"
#include "actions/compoundTask.h"
#include "ros/ros.h"
#include <ros/package.h>
#include "visualization_msgs/MarkerArray.h"
#include <string>
#include <vector>
#include "std_msgs/String.h"
#include "agents/entity.h"
#include "imhus/AsyncAction_msgs.h"

namespace actions
{
    class AsyncActionRes : public CommandAction
    {
        public:
            AsyncActionRes(actions::CompoundTask* task, agents::Entity*);

            virtual void execute();
            void shutDown();

        private:
            void triggerCB(imhus::AsyncAction_msgs);
            // void triggerCB(const std_msgs::String::ConstPtr& );
            void assignAction();
            void triggerAction(agents::Entity*);
            void pubRviz(float, float, std::string, float width);

            agents::Entity* entities_;
            CompoundTask *task_;

            //// Variables ////
            const float max_radius=3;
            const int time_out=30;
            
            imhus::AsyncAction_msgs req_info;
            bool listening_ = false;

		    ros::NodeHandle nh_;
            ros::Subscriber trigger_sub_;
            ros::Publisher marker_pub_;

            //Rviz
            visualization_msgs::MarkerArray markers_;

    };
}

#endif