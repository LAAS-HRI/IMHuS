#ifndef COMPOUNDTASK
#define COMPOUNDTASK

#include "actions/commandAction.h"
#include <vector>

namespace actions
{
    class CompoundTask
    {
        public:
        CompoundTask(std::string);

        std::string getID(){return id_;};
        std::vector<CommandAction*> getActions(){return actions_;};
        std::vector<CommandAction*>::iterator getIt(){return actions_.begin();};

        void add(CommandAction*);
        void execute(agents::Entity*);
        // CompoundTask* deepCopy();

        private:
        std::string id_;
        std::vector<CommandAction*> actions_;
    };
}


#endif