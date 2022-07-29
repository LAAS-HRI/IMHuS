#include <string>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tinyxml.h"
#include "ros/package.h"


class BossUI
{
    public:
        BossUI();

        void print();

    private:
        void readXml();
        void ready();
        void printChoice();
        void printAgents();
        void printScenarios();
        void publishScenario(std::string);

        std::vector<std::string> agents_;
        std::vector<std::string> scenarios_;

        ros::NodeHandle nh_;
        ros::Publisher bossUI_pub_; std_msgs::String msg_;

        int i=0, choice;
        std::string input;

        //Read Xml
        std::string map_name_;
        TiXmlDocument *doc_;
};