#ifndef PRUEBASC___IDLESTATE_H
#define PRUEBASC___IDLESTATE_H

#include "entityState.h"
#include "map/pose.h"
// #include "agents/entity.h"

namespace agents
{

class IdleState : public EntityState
{
    public:
        IdleState(Entity* pEntity);
        ~IdleState(){};

        void goTo(Pose, SocialSpace*);
        void wait(float);
        void wait(std::string, std::string);
        bool acceptAsyncTask();
        std::string getStateStr(){return "idle";};

};

}


#endif 
