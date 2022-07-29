#ifndef PRUEBASC___ConflictSTATE_H
#define PRUEBASC___ConflictSTATE_H

#include "entityState.h"
#include "map/pose.h"
// #include "agents/entity.h"

namespace agents
{

class ConflictState : public EntityState
{
    public:
        ConflictState(Entity* pEntity);
        ~ConflictState(){};

        void goTo(Pose, SocialSpace*);
        void wait(float);
        void wait(std::string, std::string);
        bool acceptAsyncTask(){return false;};
        std::string getStateStr(){return "in conflict !";};

    private:
        ros::Time init_;

};

}


#endif 