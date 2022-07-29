#ifndef SOCPOSE_
#define SOCPOSE_

#include <string>
#include "pose.h"

class SocialPose : public Pose
{
    public:
        SocialPose(){};

        bool free=true;
        bool assigned=false;
    private:
        

};

#endif