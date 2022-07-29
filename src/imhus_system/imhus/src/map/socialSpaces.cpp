#include "map/socialSpaces.h"

void SocialSpaces::add(SocialSpace* p)
{
    this->social_spaces.push_back(p);
}

std::tuple<bool, SocialSpace*> SocialSpaces::get(std::string id)
{
    for(auto sp : social_spaces)
    {
        if(sp->getID() == id) return std::make_tuple(true, sp);
    }
    return std::make_tuple(false, nullptr);
}