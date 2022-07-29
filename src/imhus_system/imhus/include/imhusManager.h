#ifndef DEF_imhus_systemM
#define DEF_imhus_systemM

#include "ros/ros.h"
#include <ros/package.h>
#include <string>
#include <vector>
#include <math.h>
#include <future> //threaad
// #include "task.h"
// #include "geometry_msgs/Twist.h"
// #include "geometry_msgs/Pose2D.h"
// #include "imhus/ComputePlan.h"
// #include "imhus/Goal.h"
// #include "std_srvs/Empty.h"
// #include "imhus/ActionBool.h"
// #include "imhus_navigation/PlaceRobot.h"
// #include "std_msgs/Int32.h"
// #include "std_msgs/String.h"
// #include "nav_msgs/Path.h"
// #include "nav_msgs/GetPlan.h"
// #include <actionlib/client/simple_action_client.h>
// #include <actionlib_msgs/GoalStatusArray.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "move_base_msgs/MoveBaseActionGoal.h"
// #include <tf2/LinearMath/Quaternion.h>
// #include "visualization_msgs/Marker.h"
// //#include "manipulate_path.hpp"
// #include "types.h"
#include <tinyxml.h>
#include <iostream>
#include <vector>
#include <thread>
//imhus
#include "imhus.h"


class ImhusManager
{
public:
	ImhusManager();

	void update();

private:

////////// METHODS //////////

	void runScenario(std::string);

////////// ATTRIBUTES //////////

	Imhus imhus{"root"};

	// Subscribers //
	ros::Subscriber sub_new_goal_;
	void newGoalCallback(const std_msgs::String::ConstPtr& msg);

	// Publishers //
	ros::Publisher pub_log_; std_msgs::String msg_; // to publish easily on log

	//// Variables ////
	ros::NodeHandle nh_;

	bool running_=false;
	std::string scenario_running_;
};

#endif
