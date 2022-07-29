#include "imhusManager.h"

///////////////////////////// SUPERVISOR /////////////////////////////////

ImhusManager::ImhusManager()
{
	// Subscribers
	// sub_new_goal_ = nh_.subscribe("new_goal", 100, &ImhusManager::newGoalCallback, this);
	sub_new_goal_ = nh_.subscribe("/boss_ui", 1, &ImhusManager::newGoalCallback, this);

	// Publishers
	pub_log_ = nh_.advertise<std_msgs::String>("log", 100);

}

void ImhusManager::runScenario(std::string req)
{
	running_=true;
	scenario_running_ = req;

	this->imhus.execute(req);
	
	running_=false;
}



void ImhusManager::newGoalCallback(const std_msgs::String::ConstPtr &msg)
{
	if(!running_)
	{
		this->runScenario(msg->data);
	}
	else
	{
		this->imhus.stop(scenario_running_);
		this->runScenario(msg->data);
	}
}

void ImhusManager::update()
{
	this->imhus.update();
}

/////////////////////////////// MAIN /////////////////////////////////////

int main(int argc, char **argv)
{
	ros::init(argc, argv, "InHuS_Manager");

	ImhusManager imhusManager;

	ros::Rate loop_rate(20);

	while (ros::ok())
	{
		ros::spinOnce();
		// imhusManager.update();
		loop_rate.sleep();
	}

	return 0;
}

//////////////////////////////////////////////////////////////////////////
