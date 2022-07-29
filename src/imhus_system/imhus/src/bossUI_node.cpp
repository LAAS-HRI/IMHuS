#include "bossUI.h"

#define XML "goals.xml"

BossUI::BossUI()
{
    bossUI_pub_ = nh_.advertise<std_msgs::String>("/boss_ui", 10);
    this->readXml();
}

void BossUI::readXml()
{
    //INIT READING
    nh_.param(std::string("map_name"), map_name_, std::string("elevator"));

    std::string goal_file = ros::package::getPath("imhus") + "/config/goals/" + map_name_ + "/" + XML;
    doc_ = new TiXmlDocument(goal_file);
    if (!doc_->LoadFile())
        ROS_ERROR("Boss: Failed to load %s. Error : %s", goal_file.c_str(), doc_->ErrorDesc());
    // Check if file is corresponding with map_name
    TiXmlHandle docHandle(doc_);
    TiXmlElement *l_map = docHandle.FirstChild("map_name").ToElement();
    std::string map_name_read = "";
    if (NULL != l_map->Attribute("name"))
        map_name_read = l_map->Attribute("name");
    if (map_name_read != map_name_)
        ROS_ERROR("Boss: Goals file mismatches the map_name");

    //READ AGENTS
    TiXmlElement *agent = docHandle.FirstChild("agents").FirstChild().ToElement();
    while (agent)
    {
        if (NULL != agent->Attribute("id"))
        {
            agents_.push_back(agent->Attribute("id"));
        }
        else
        {
            ROS_ERROR("Boss : You forgot to specify id of a human in the xml, cannot create it.");
        }
        agent = agent->NextSiblingElement();
    }

    //READ SCENARIOS
    TiXmlElement *scenario_xml = docHandle.FirstChild("goals").FirstChild("scenarios").FirstChild().ToElement();

    while (scenario_xml)
    {
        scenarios_.push_back(scenario_xml->Value());
        scenario_xml = scenario_xml->NextSiblingElement();
    }

    this->ready();
}

void BossUI::ready()
{
    std::cout << std::endl;
	std::cout << "==========================" << std::endl;
	std::cout << "=========> BOSS <=========" << std::endl;
	std::cout << "==========================" << std::endl;
	std::cout << std::endl;
}

void BossUI::print()
{
    this->printAgents();
    this->printChoice();

    
}

void BossUI::printChoice()
{
    std::cout << "==========================" << std::endl;
    std::cout << "========  CHOICE  ========" << std::endl;
    std::cout << "==========================" << std::endl;
    std::cout << "1 -> Scenarios" << std::endl;
    std::cin >> input;
    try
    {
        choice = std::stoi(input);
        switch (choice)
        {
        case 1:
            this->printScenarios();
            break;
        case 2:
            //
            break;
        
        default:
            break;
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    
}

void BossUI::printAgents()
{
    i = 0;
    std::cout << std::endl;
    std::cout << " Agents " << std::endl;
    for(std::string agent : agents_)
    {
        i++;
        std::cout << i << " -> " << agent << std::endl;
    }
}

void BossUI::printScenarios()
{
    i = 0;
    std::cout << std::endl;
    std::cout << " Scenarios " << std::endl;
    for(std::string sc : scenarios_)
    {
        i++;
        std::cout << i << " -> " << sc << std::endl;
    }

    std::cin >> input;
    try
    {
        choice = std::stoi(input);
        if(choice > 0 && choice < scenarios_.size()+1)
        {
            this->publishScenario(scenarios_[choice-1]);
        }
    }catch(const std::string& ex)
    {
        ROS_INFO("Your input was not read, try again. \n %s", ex);
    }
}

void BossUI::publishScenario(std::string sc)
{
    msg_.data = sc;
    bossUI_pub_.publish(msg_);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "bossUI");

	BossUI boss;

	while(ros::ok())
	{
		boss.print();
	}
}