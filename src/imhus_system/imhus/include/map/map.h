#ifndef MAPOBJECT
#define MAPOBJECT

#include <string>
#include <vector>
#include "map/objects.h"
#include "map/poses.h"
#include "socialSpaces.h"

class Map
{
    public:
    Map();
    
    Objects objects;
    Poses poses;
    SocialSpaces social_spaces;

};

#endif