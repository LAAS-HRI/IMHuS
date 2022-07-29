#include "map/objects.h"

Object Objects::get(std::string id)
{
    for(auto o : objects)
    {
        if(o.getID() == id) return o;
    }
    ROS_ERROR("objetcs::get() was not able to find object: %s. Check id.", id.c_str());
    return objects[0];
}

void Objects::add(Object new_obj)
{
    if(!this->isAlreadyHere(new_obj)) objects.push_back(new_obj);
}


bool Objects::isAlreadyHere(Object obj)
{
    for(auto o : objects)
    {
        if(o.getID() == obj.getID()) return true;
    }
    return false;
}