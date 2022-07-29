#include "actions/commandAction.h"
#include "ros/ros.h"

#include <string>
#include <std_msgs/String.h>

namespace actions{
    
class PublishAction : public CommandAction
{
    public:
        PublishAction();
        PublishAction(std::string topic, std::string msg);

        void execute();

    private:
        std::string topic_;
        std::string msg_;

        ros::NodeHandle nh_;

};


}