#ifndef DEF_Entity
#define DEF_Entity

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include <vector>
#include "map/pose.h"
#include "agents/state/entityState.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
// #include "map/socialSpace.h"



//////////////////////////////////////////////////

////////////////// Entity ///////////////////////
class SocialSpace; //forward declarations to avoid circular dep
namespace agents
{
    class EntityState;
    class Entity
    {
    public:
        Entity();
        std::string getID(){return id_;};
        float getRadius(){return radius;};
        virtual Pose getPose()=0;

        virtual void add(Entity*)=0;
        virtual Entity* getEntityPtr(std::string)=0;
        virtual std::vector<Entity*>::iterator getMembersIt()=0;
        virtual std::vector<Entity*> getMembers()=0;

        void setState(EntityState*);
        void logInfo();

        //Actions
        virtual void goTo(Pose, SocialSpace*)=0;
        virtual void wait(float)=0;
        virtual void wait_event(std::string, std::string)=0;
        virtual void publish(std::string, std::string)=0;
        virtual bool acceptAsyncTask()=0;

        virtual void update()=0;
        virtual bool isAlreadyHere(std::string id)=0;
        virtual void sendPlan(nav_msgs::Path)=0;


        void spinOnce();
        //these are used by go to action (human) but are defined here to avoid
        //the delay of advertising a topic
         // Publishers //
		ros::Publisher pub_goal_move_base_; 
        
        std::string event; 
        //maybe they should be protected 
        void freeCB(std_msgs::String); void setFreeSub(std::string);
        ros::Subscriber free_sub_; //for wait event
        ros::Publisher free_pub_; //for publish action topic
        ros::Publisher plan_pub_;
        ros::Publisher cmd_pub_;

    protected:
        ros::NodeHandle nh_;
        std::string id_;
        EntityState* state_;
        float radius=0.3;

        ros::Subscriber pose_sub_;
        virtual void poseCB(nav_msgs::Odometry)=0; //TODO changed to !trackedagents!
        Pose pose_;
     
    };
        
}

#endif