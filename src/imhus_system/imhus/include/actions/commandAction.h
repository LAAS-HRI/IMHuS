//abstract class for the command pattern applied to actions
#ifndef COMMAD_ACTION_
#define COMMAD_ACTION_
#include "agents/group.h"
#include "ros/ros.h"


//////////////////////////////////////////////////

////////////////// Action /////////////////
namespace actions
{
    class CommandAction
    {
    public:
        CommandAction();
        ~ CommandAction(){};

        //methods
        void set(agents::Entity* entity); 
        // void setID(std::string id){id_=id;};
        // std::string getID(){return id_;};
        std::string getEntityID(){return entity_->getID();};
        virtual void execute(){ROS_ERROR("Trying to execute abstract class CommandAction, you forgot to cast the action back in its derived class.");};

    protected:
        agents::Entity* entity_;
        // std::string id_;

    };
}

#endif