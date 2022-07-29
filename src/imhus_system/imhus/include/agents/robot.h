#ifndef ROBOTCLASS
#define ROBOTCLASS

#include "agents/agent.h"
#include <string>
#include "ros/ros.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "std_msgs/String.h"


//////////////////////////////////////////////////

////////////////// Robot /////////////////
class SocialSpace; //forward declarations to avoid circular dep
namespace agents
{
    class Robot : public Agent
    {
    public:
        Robot();
        Robot(std::string id);

        ////virtuals inherited, should not be used////
        void add(Entity*){ROS_WARN("A robot cannot add an entity.");};
        Entity* getEntityPtr(std::string){return this;};
        std::vector<Entity*>::iterator getMembersIt(){ROS_WARN("A robot cannot return an iterator.");};
        std::vector<Entity*> getMembers(){ROS_WARN("A robot cannot return an iterator.");};
        /////////////////////

        bool isAlreadyHere(std::string id);

        Pose getPose(){ros::spinOnce(); return pose_;};
        //Actions
        void goTo(Pose p, SocialSpace* sp){this->state_->goTo(p, sp);};
        void wait(float t){this->state_->wait(t);};
        void wait_event(std::string topic, std::string event){this->state_->wait(topic, event);};
        void publish(std::string, std::string);
        bool acceptAsyncTask(){this->state_->acceptAsyncTask();};
        
        void sendPlan(nav_msgs::Path){};
        void update();

    private:
        void init();
        //ros
        ros::NodeHandle nh_;
        void poseCB(nav_msgs::Odometry);
    };
}

#endif