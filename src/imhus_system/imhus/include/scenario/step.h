// #include "agents/group.h"
// #include "agents/agent.h"
#include "agents/entity.h"
#include "actions/compoundTask.h"
#include <string>

namespace scenario
{

class Step
{
    public:
    Step(std::string id);

    void add(agents::Entity *entity, actions::CompoundTask *task);
    void execute();

    std::vector<agents::Entity*> p_entity;
    std::vector<actions::CompoundTask*> p_task;

    std::string id;

    private:
        // void addEntityList(agents::Entity* );
        // std::vector<agents::Entity*> entities_list;
        // std::vector<std::vector<actions::CompoundTask*>> tasks_list;

};

}