#include "agents/robot.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>

using namespace agents;

Robot::Robot()
{
    ROS_WARN("No idea given for a robot.");
    id_ = "robot_nameless";
    this->init();
}

Robot::Robot(std::string id)
{
    id_ = id;
    this->init();
}

void Robot::init()
{
    // Publishers
	pub_goal_move_base_ = nh_.advertise<move_base_msgs::MoveBaseActionGoal>("/TODO/robot_move_base/goal", 100);
    //Subscribers
    pose_sub_ = nh_.subscribe("/odom", 10, &Robot::poseCB, this);

    // this->state_ = new IdleState(this);
    this->radius = 0.3;
}

bool Robot::isAlreadyHere(std::string id)
{
    return id==id_;
}

void Robot::publish(std::string topic, std::string msg)
{
    
    free_pub_ = nh_.advertise<std_msgs::String>(topic, 100);

    if (topic.empty())
    {
        ROS_WARN("Topic to publish not specified");
    }
    else if (msg.empty())
    {
        ROS_WARN("Msg to publish not specified");
    }
    else
    {
        ROS_INFO("Agent %s will publish : %s, on topic : %s", id_.c_str(), msg.c_str(), topic.c_str());
    }
    //TODO reduce this time to minimum
    ros::Duration(.7).sleep();

    std_msgs::String ros_msg;
    ros_msg.data = msg;
    free_pub_.publish(ros_msg);
}

void Robot::update()
{
    
}

void Robot::poseCB(nav_msgs::Odometry msg)
{
    pose_.setX(msg.pose.pose.position.x);
    pose_.setY(msg.pose.pose.position.y);
    pose_.setQuat(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z);
}
