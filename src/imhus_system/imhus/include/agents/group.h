#ifndef GROUP_
#define GROUP_

// #include "agents/entity.h"
#include "agents/group.h"
#include "agents/human.h"
#include "ros/ros.h"
#include <thread>
#include <vector>

class SocialSpace; //forward declarations to avoid circular dep
namespace agents
{
    class Group : public Entity
    {
    public:
        Group();
        Group(std::string id);

        Pose getPose(){return pose_;};

        std::string getID(){return id_;};
        void add(Entity*);
        Entity* getEntityPtr(std::string);
        std::vector<Entity*>::iterator getMembersIt();
        std::vector<Entity*> getMembers();
        
        bool isAlreadyHere(std::string id);

        //Actions
        void goTo(Pose p, SocialSpace*);
        void wait(float);
        void wait_event(std::string,std::string);
        void publish(std::string, std::string){};
        bool acceptAsyncTask(){};

        void sendPlan(nav_msgs::Path p);
        void update();

    private:
        std::vector<Entity*> members_;
        void poseCB(nav_msgs::Odometry);
        ros::Time init_time_;
    };

}




#endif