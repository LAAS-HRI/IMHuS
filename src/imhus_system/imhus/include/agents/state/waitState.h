#ifndef PRUEBASC___WAITSTATE_H
#define PRUEBASC___WAITSTATE_H

#include "entityState.h"
#include "map/pose.h"
// #include "agents/entity.h"

namespace agents
{

class WaitState : public EntityState
{
    public:
        WaitState(Entity* pEntity);
        ~WaitState(){};

        void goTo(Pose, SocialSpace*);
        void wait(float);
        void wait(std::string, std::string);
        bool acceptAsyncTask(){return false;};
        std::string getStateStr(){return "waiting";};
};

}


#endif 