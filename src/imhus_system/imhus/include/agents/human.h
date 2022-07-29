#ifndef HUMAN
#define HUMAN

#include "agents/agent.h"
#include <string>
#include "ros/ros.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "std_msgs/String.h"
#include "nav_msgs/Path.h"
#include "visualization_msgs/MarkerArray.h"


//////////////////////////////////////////////////

////////////////// GoToAction /////////////////
class SocialSpace; //forward declarations to avoid circular dep
namespace agents
{
    class Human : public Agent
    {
    public:
        Human();
        Human(std::string id);

        ////virtuals inherited, should not be used////
        void add(Entity*){ROS_WARN("A human cannot add an entity.");};
        Entity* getEntityPtr(std::string){return this;};
        std::vector<Entity*>::iterator getMembersIt(){ROS_WARN("A human cannot return an iterator.");};
        std::vector<Entity*> getMembers(){ROS_WARN("A human cannot return an iterator.");};
        /////////////////////

        bool isAlreadyHere(std::string id);

        //Actions
        void goTo(Pose p, SocialSpace* sp){this->state_->goTo(p, sp);};
        void wait(float t){this->state_->wait(t);};
        void wait_event(std::string topic, std::string event){this->state_->wait(topic, event);};
        void publish(std::string, std::string);
        bool acceptAsyncTask(){this->state_->acceptAsyncTask();};

        void sendPlan(nav_msgs::Path);
        void update();
        Pose getPose();

    private:
        void init();
        //ros
        ros::NodeHandle nh_;
        void poseCB(nav_msgs::Odometry);
        void pubRviz();

        //Rviz
        visualization_msgs::MarkerArray marker_;
        ros::Publisher marker_pub_;
    };
}

#endif