#include "scenario/steps.h"

using namespace scenario;

// Step Steps::get(std::string id)
// {
//    //iterator?
//     return steps_[0];
// }

void Steps::add(Step* new_s)
{
    steps_.push_back(new_s);
}


