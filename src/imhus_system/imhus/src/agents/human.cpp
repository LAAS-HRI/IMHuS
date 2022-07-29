#include "agents/human.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>

using namespace agents;

Human::Human()
{
    ROS_WARN("No idea given for a human.");
    id_ = "human_nameless";
    this->init();
}

Human::Human(std::string id)
{
    id_ = id;
    this->init();
}

void Human::init()
{
    // Publishers
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/imhus/output/"+this->getID()+"/pref_vel", 1); //ORCA
    plan_pub_ = nh_.advertise<nav_msgs::Path>("/imhus/output/" + this->getID() + "/plan", 1); //WO_ORCA
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/imhus/output/"+getID()+"/marker_human_pose", 1);

    //Subcriber
	pose_sub_ = nh_.subscribe("/imhus/input/"+getID()+"/odom", 100, &Human::poseCB, this);
    
    // this->state_ = new IdleState(this);
    this->radius = 0.3;
    //init rviz marker
    visualization_msgs::Marker marker;
	marker.header.frame_id = "map";
    tf2::Quaternion q;
	q.setRPY(0,0,0);
	marker.pose.orientation.w = q.w();
	marker.pose.orientation.x = q.x();
	marker.pose.orientation.y = q.y();
	marker.pose.orientation.z = q.z();
    //ARROW
    marker.type = 0;
	marker.id = 0;
	marker.pose.position.z = 0.1;
	marker.scale.x = 0.6;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.color.r = 1;
	marker.color.g = 1;
	marker.color.b = 0;
	marker.color.a = 1;
	marker_.markers.push_back(marker);
    // CYLINDER
	marker.type = 3;
	marker.id = 1;
	marker.pose.position.z = 0.9;
	marker.scale.x = 0.4;
	marker.scale.y = 0.4;
	marker.scale.z = 1.8;
	marker.color.r = 1;
	marker.color.g = 1;
	marker.color.b = 0;
	marker.color.a = 1;
	marker_.markers.push_back(marker);
    //text
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = 2;
    marker.pose.position.z = 2.5;
    marker.scale.z = 0.4; //font size
    // marker.scale.x = 10; //not used for text
    // marker.scale.y = 10;
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.text = getID();
    marker.color.a = 1;
    //
	marker_.markers.push_back(marker);
}

bool Human::isAlreadyHere(std::string id)
{
    return id==id_;
}

void Human::publish(std::string topic, std::string msg)
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

void Human::poseCB(nav_msgs::Odometry msg)
{
    pose_.setX(msg.pose.pose.position.x);
    pose_.setY(msg.pose.pose.position.y);
    pose_.setQuat(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
    this->pubRviz();
}

void Human::pubRviz()
{
    tf2::Quaternion q;
    float x=pose_.getX(), y=pose_.getY();
	
	marker_.markers[0].pose.position.x = x;
	marker_.markers[0].pose.position.y = y;
	marker_.markers[1].pose.position.x = x;
	marker_.markers[1].pose.position.y = y;
	marker_.markers[2].pose.position.x = x;
	marker_.markers[2].pose.position.y = y;
    marker_.markers[2].text = getID() + "\n"+this->state_->getStateStr();
	q.setRPY(0,0,pose_.getTheta());
	marker_.markers[0].pose.orientation.w = q.w();
	marker_.markers[0].pose.orientation.x = q.x();
	marker_.markers[0].pose.orientation.y = q.y();
	marker_.markers[0].pose.orientation.z = q.z();
    marker_pub_.publish(this->marker_);
}

void Human::sendPlan(nav_msgs::Path plan)
{
    plan_pub_.publish(plan);
}

void Human::update()
{
    ros::spinOnce();
    this->pubRviz();
}

Pose Human::getPose()
{
    ros::spinOnce();
    return pose_;
}

// void Human::freeCB(std_msgs::String msg)
// {
//     event = msg.data;
// }

