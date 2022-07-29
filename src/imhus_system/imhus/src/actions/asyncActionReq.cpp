#include "actions/asyncActionReq.h"

using namespace actions;

AsyncActionReq::AsyncActionReq(std::string task)
:taskID_(task)
{
    trigger_pub_ = nh_.advertise<imhus::AsyncAction_msgs>("/async_actions_topic", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/marker_async_req", 1);

    //RVIZ
    visualization_msgs::Marker marker;
	marker.header.frame_id = "map";
    tf2::Quaternion q;
	q.setRPY(0,0,0);
	marker.pose.orientation.w = q.w();
	marker.pose.orientation.x = q.x();
	marker.pose.orientation.y = q.y();
	marker.pose.orientation.z = q.z();
    //Square
    marker.type = 1;
	marker.id = 0;
	marker.pose.position.z = 3.;
	marker.scale.x = 2.7;
	marker.scale.y = 0.8;
	marker.scale.z = 0.1;
	marker.color.r = 1;
	marker.color.g = 1;
	marker.color.b = 1;
	marker.color.a = 1;
	markers_.markers.push_back(marker);
    //text
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = 1;
    marker.pose.position.z = 2.75;
    marker.scale.z = 0.4; //font size
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1;
    //
	markers_.markers.push_back(marker);
}

void AsyncActionReq::execute()
{
    req_info.header.stamp = ros::Time::now();
    req_info.header.frame_id = "map";
    req_info.taskID = taskID_;
    req_info.agent = entity_->getID();
    req_info.pose.x = entity_->getPose().getX();
    req_info.pose.y = entity_->getPose().getY();
    req_info.pose.theta = entity_->getPose().getTheta();

    trigger_pub_.publish(req_info);
    this->pubRviz(entity_->getPose().getX(),entity_->getPose().getY(),"Request:\n" + taskID_,3);

    ROS_WARN("%s requested an async action for task : %s", entity_->getID().c_str(), taskID_.c_str());
}

void AsyncActionReq::pubRviz(float x, float y, std::string text, float width)
{
    // //rviz	//will appear on top of agent asking the request
    // visualization_msgs::Marker marker;
    // marker.header.frame_id = "map";
    // tf2::Quaternion q;
	// q.setRPY(0,0,0);
	// marker.pose.orientation.w = q.w();
	// marker.pose.orientation.x = q.x();
	// marker.pose.orientation.y = q.y();
	// marker.pose.orientation.z = q.z();

    // //text
    // marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    // marker.action = visualization_msgs::Marker::ADD;
    // marker.id = 0;
    // marker.pose.position.z = 3;
    // marker.scale.z = 0.4; //font size
    // // marker.scale.x = 15;
    // // marker.scale.y = 15;
    // marker.color.r = 0.0f;
	// marker.color.g = 0.0f;
	// marker.color.b = 0.0f;
    // marker.color.a = 1;
    // marker.lifetime = ros::Duration(5.0);

	// marker.pose.position.x = entity_->getPose().getX();
	// marker.pose.position.y = entity_->getPose().getY();
    // marker.text = "Request:\n" + taskID_;
    markers_.markers[0].lifetime = ros::Duration(2.0);
    markers_.markers[1].lifetime = ros::Duration(2.0);
    markers_.markers[0].pose.position.x = x;
    markers_.markers[0].pose.position.y = y;
    markers_.markers[0].scale.x = width;
    markers_.markers[1].pose.position.x = x;
    markers_.markers[1].pose.position.y = y;
    markers_.markers[1].text = text;

    marker_pub_.publish(markers_);
}