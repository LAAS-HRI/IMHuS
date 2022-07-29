#include "actions/asyncActionRes.h"

using namespace actions;

AsyncActionRes::AsyncActionRes(actions::CompoundTask* task, agents::Entity* imhus_entities)
:task_(task), entities_(imhus_entities)
{
    trigger_sub_ = nh_.subscribe("/async_actions_topic", 1, &AsyncActionRes::triggerCB, this);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/marker_async_res", 100);
    //TODO publish a msg for the user to know when the req is accepted or not

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
	marker.scale.x = 3;
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
    marker.pose.position.z = 3.1;
    marker.scale.z = 0.4; //font size
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1;
    //
	markers_.markers.push_back(marker);
}

void AsyncActionRes::execute()
{
    ROS_INFO("An AsyncAction can now be triggered.");
    listening_ = true;
    while(listening_)
    {
        ros::spinOnce();
    }
}

void AsyncActionRes::shutDown()
{
    listening_ = false;
}

void AsyncActionRes::assignAction()
{
    bool answer=false;
    agents::Entity* best_;
    float score=9999;

    for(auto entity : entities_->getMembers())
    {
        if(entity->getID() != req_info.agent && entity->acceptAsyncTask())
        {
            float d=sqrt(pow(req_info.pose.x-entity->getPose().getX(),2)+pow(req_info.pose.y-entity->getPose().getY(),2));
            if(d < max_radius)
            {
                answer=true;
                if(d<score) //keep if better than what we found
                {
                    score = d;
                    best_ = entity;
                }
            }
        }
    }  

    //cases
    if(!answer && ros::Time::now() > req_info.header.stamp + ros::Duration(time_out)) 
    {
        agents::Entity* entity = entities_->getEntityPtr(req_info.agent);
        if(entity!=nullptr)
        {
            this->pubRviz(entity->getPose().getX(), entity->getPose().getY()+1, "Request was not answered !", 4.5);
            ROS_ERROR("No one was able to answer to the resquested asyncAction %s.", task_->getID().c_str());
        }
        else
        {
            ROS_ERROR("Entity %s is unknown !", req_info.agent.c_str());
        }
        //if we want to let the response action open for a nother request we need to listen_ again :
        this->listening_ = true; 
        return;
    }
    else if(!answer && ros::Time::now() <= req_info.header.stamp + ros::Duration(time_out))
    {
        agents::Entity* entity = entities_->getEntityPtr(req_info.agent);
        if(entity!=nullptr)
        {
            this->pubRviz(entity->getPose().getX(), entity->getPose().getY()+1, "Request cannot be answered now.\nRetrying in 5 sec...", 6);
            ROS_WARN("No one was able to answer to the resquested asyncAction %s. Retrying in 5 sec...", task_->getID().c_str());
        }
        else
        {
            ROS_ERROR("Entity %s is unknown !", req_info.agent.c_str());
        }
        ros::Duration(5).sleep();
        this->assignAction();
    }
    else
    {
        this->pubRviz(best_->getPose().getX(), best_->getPose().getY()+1, best_->getID()+" accepts request:\n" + task_->getID(), 4);
        ROS_WARN("An AsyncAction has been triggered on entity %s to execute task %s !", best_->getID().c_str(),task_->getID().c_str());
        this->triggerAction(best_);
    }
}

void AsyncActionRes::triggerAction(agents::Entity* entity_ptr)
{
    task_->execute(entity_ptr);
    ROS_WARN("AsyncAction (%s) done.", task_->getID().c_str());
}

void AsyncActionRes::triggerCB(imhus::AsyncAction_msgs msg)
{
    if(this->listening_ && msg.taskID == task_->getID())
    {
        this->listening_ = false;
        req_info = msg;
        this->assignAction();
    } 
}

void AsyncActionRes::pubRviz(float x, float y, std::string text, float width)
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
    // marker.scale.z = 0.4;
    // // marker.scale.x = 15;
    // // marker.scale.y = 15;
    // marker.color.r = 0.0f;
	// marker.color.g = 0.0f;
	// marker.color.b = 0.0f;
    // marker.color.a = 1;
    // marker.lifetime = ros::Duration(5.0);

	// marker.pose.position.x = x;
	// marker.pose.position.y = y;
    // marker.text = text;
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