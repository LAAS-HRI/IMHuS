#include <string>
#include <vector>
#include "ros/ros.h"
#include "scenario/step.h"

namespace scenario
{

class Steps
{
    public:
    Steps(){};

    void add(Step*);
    std::vector<Step*> getSteps(){return steps_;};
    // std::vector<Step>::iterator getIt(){return steps_.begin();};

    private:
    std::vector<Step*> steps_;

};

}