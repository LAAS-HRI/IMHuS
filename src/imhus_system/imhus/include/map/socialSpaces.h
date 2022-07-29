#ifndef SOCIALSPACES
#define SOCIALSPACES

#include <vector>
#include <string>
#include "socialSpace.h"

class SocialSpaces
{
    public:
        SocialSpaces(){};

        void add(SocialSpace*);
        std::tuple<bool, SocialSpace*> get(std::string);



    private:
        std::vector<SocialSpace*> social_spaces;
};


#endif