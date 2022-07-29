#include "agents/entity.h"
#include <string>
#include <vector>
#include "agents/state/idleState.h"

using namespace agents;

Entity::Entity()
{
    //should be moved to human, robot and group constructor ?
    this->state_ = new IdleState(this);
}

void Entity::setState(EntityState* pState)
{
    this->state_ = pState;
    //the line below is just to force the update of the markers in rviz
    this->update();
}

void Entity::setFreeSub(std::string topic)
{
    event = "";
    // try
    // {
    free_sub_ = nh_.subscribe(topic, 10, &Entity::freeCB, this);       
    // }
    // catch(const std::exception& e)
    // {
    //     std::cerr << e.what() << '\n';
    // }
}

void Entity::spinOnce()
{
    ros::spinOnce();
}

void Entity::freeCB(std_msgs::String msg)
{
    event = msg.data;
}

// Pose Entity::getPose()
// {
//     ros::spinOnce();
//     return pose_;
// }

// void Entity::poseCB(nav_msgs::Odometry msg)
// {
//     ROS_INFO("odom");
//     pose_.setX(msg.pose.pose.position.x);
//     pose_.setY(msg.pose.pose.position.y);
//     pose_.setQuat(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z);
//     ROS_INFO("pos : %.1f", pose_.getX());
// }