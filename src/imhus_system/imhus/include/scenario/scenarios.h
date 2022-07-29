#include "scenario/scenario.h"
#include <string>
#include <vector>

namespace scenario
{

class Scenarios
{
    public:
    Scenarios(){};

    void add(Scenario*);
    Scenario* get(std::string);
    std::vector<Scenario*> getScenarios(){return scenarios_;};

    std::vector<Scenario*> scenarios_;

    private:
    bool isAlreadyHere(Scenario*);
};

}