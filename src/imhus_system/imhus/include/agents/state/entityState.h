#ifndef PRUEBASC___ENTITYSTATE_H
#define PRUEBASC___ENTITYSTATE_H

#include "agents/entity.h"
#include "map/pose.h"
#include "ros/ros.h"


class SocialSpace; //forward declarations to avoid circular dep
namespace agents
{
    class Entity;
    class EntityState
    {
        public:
            virtual void wait(float)=0;
            virtual void wait(std::string, std::string)=0;
            virtual void goTo(Pose, SocialSpace*)=0;
            virtual bool acceptAsyncTask()=0;
            virtual std::string getStateStr()=0;



        protected:
            Entity* entity_;
    };
}


#endif //PRUEBASC___ENTITYSTATE_H
