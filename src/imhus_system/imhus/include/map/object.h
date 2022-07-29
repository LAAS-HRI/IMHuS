#ifndef OBJECT_
#define OBJECT_

#include <string>
#include "map/pose.h"

class Object
{
    public:
    Object(std::string, Pose);
    std::string getID(){return id_;};
    Pose getPose(){return pose_;};

    private:
    std::string id_;
    Pose pose_;
};

#endif