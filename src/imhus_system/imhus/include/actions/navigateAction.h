#ifndef NAVIGATE_ACTION
#define NAVIGATE_ACTION

//abstract class for the command pattern applied to actions
#include "actions/commandAction.h"


//////////////////////////////////////////////////

////////////////// NavigateAction /////////////////
namespace actions
{
    class NavigateAction : public CommandAction
    {
        public:
        NavigateAction();
        ~ NavigateAction(){};

    };
}

#endif