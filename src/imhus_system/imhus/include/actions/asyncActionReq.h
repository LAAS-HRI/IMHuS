#ifndef ASYNC_ACTIONREQ
#define ASYNC_ACTIONREQ

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
    class AsyncActionReq : public CommandAction
    {
        public:
            AsyncActionReq(std::string task);

            virtual void execute();

        private:
            void pubRviz(float x, float y, std::string text, float width);
            //// Variables ////
            std::string taskID_;
            imhus::AsyncAction_msgs req_info;

		    ros::NodeHandle nh_;
            ros::Publisher trigger_pub_;
            ros::Publisher marker_pub_;

            //Rviz
            visualization_msgs::MarkerArray markers_;

    };
}

#endif