#include <string>
#include <vector>

#include "scenario/steps.h"

namespace scenario
{

class Scenario
{
    public:
    Scenario(){};
    Scenario(std::string _id){id=_id;};

    void execute();

    bool stopping_request=false;

    std::string id;
    Steps steps;

};

}