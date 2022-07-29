#include "ros/ros.h"
#include "ros/package.h"
#include <tinyxml.h>
#include <string>
#include <vector>
#include "map/map.h"
#include "scenario/scenarios.h"
#include "agents/human.h"
#include "agents/robot.h"
#include "agents/group.h"
// include task and scenarios
#include "actions/compoundTasks.h"
#include "actions/goToAction.h"
#include "actions/waitAction.h"
#include "actions/publishAction.h"
#include "actions/asyncActionReq.h"
#include "actions/asyncActionRes.h"

class Imhus
{
public:
    Imhus(std::string root_id);

    void execute(std::string);
    void stop(std::string);
    void update();
    
    // attributes
    Map map;
    agents::Group entity{"InHuS_entities"};
    actions::CompoundTasks tasks;
    scenario::Scenarios scenarios;

private:
    void readXml(std::string file);
    void createAgent(std::string id, std::string type);
    bool isAlreadyHere(std::string);

    std::vector<std::string> string_split(const std::string &str);    

    // other
    ros::NodeHandle nh_;
    std::string map_name_;
    TiXmlDocument *doc_;
};