#include "actions/compoundTask.h"
#include <vector>
#include <string>


namespace actions
{
    class CompoundTasks
    {
        public:
        CompoundTasks();

        std::string getID(){return id_;};
        std::vector<actions::CompoundTask*> getTasks(){return tasks_;};
        actions::CompoundTask* getTask(std::string);

        void add(actions::CompoundTask*);
        

        private:
        std::string id_;
        std::vector<actions::CompoundTask*> tasks_;
    };
}