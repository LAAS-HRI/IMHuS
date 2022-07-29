//abstract class for the command pattern applied to actions
#include "commandAction.h"


//////////////////////////////////////////////////

////////////////// TurningAction /////////////////
namespace actions
{
    class TurningAction : public CommandAction
    {
    public:
        TurningAction();
        ~ TurningAction(){};

    private:
    };
}
