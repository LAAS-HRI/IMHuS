#include "actions/lookAtAction.h"

using namespace actions;

LookAtAction::LookAtAction()
{
    ROS_ERROR("Uncorrect call of lookataction. Use constructor with either an angle as float or a target name as string.");
}

LookAtAction::LookAtAction(float angle)
:cmd_angle_(angle)
{
    this->init();
}

LookAtAction::LookAtAction(std::string target)
:target_(target)
{
    this->init();
}

void LookAtAction::findTarget()
{
    //here we ask sim for target location and compute desired angle
    ros::spinOnce();
    int i =0;
    for(auto name : this->link_states.name)
    {
        if(name == target_)
        {
            target_x_ = this->link_states.pose[i].position.x;
            target_y_ = this->link_states.pose[i].position.y;
            target_found_ = true;
            break;
        }
        i++;
    }
    if(!target_found_) {
        ROS_ERROR("Target has not been found. Skipping LookAtAction. Please do specify an existing link to look at.");
        task_done_ = true;
    }
}

void LookAtAction::init()
{
    //this fct is to init the connection with topics
    last_check_ = ros::Time::now();
    link_states_sub_ = nh_.subscribe("/gazebo/link_states", 1000, &LookAtAction::linkStatesCB, this);
    set_link_pub_ = nh_.advertise<gazebo_msgs::LinkState>("/gazebo/set_link_state", 10);

    ang_speed_ = 1;
    target_found_ = false; //not used if target is not specified or not found in world

    ros::Duration(1).sleep();
}

void LookAtAction::linkStatesCB(gazebo_msgs::LinkStates msg)
{
    if(ros::Time::now() > last_check_ + ros::Duration(.1))
    {
        this->link_states = msg;
        last_check_ = ros::Time::now();
    }
}

gazebo_msgs::LinkState LookAtAction::getState(std::string agent)
{   
    ros::spinOnce();
    std::string link_name = agent+"::link";
    gazebo_msgs::LinkState state_;
    int i =0;
    for(auto name : this->link_states.name)
    {
        if(name == agent+"::link")
        {
            state_.link_name = agent+"::link";
            state_.pose = this->link_states.pose[i];
            state_.twist = this->link_states.twist[i];
            return state_;
        }
        i++;
    }
    //failure
    return state_;
}

void LookAtAction::execute()
{
    // if(!this->group.getMembers().empty())
    // {
        if(!target_.empty()) this->findTarget();
        while(!task_done_)
        {
            ros::spinOnce(); //very important
            this->doTask();
        }
        ROS_INFO("LookAtAction is done.");
    // }
    // else{
    //     ROS_ERROR("The group has not been set or is empty. Skipping this LookAtAction.");
    // }
}

void LookAtAction::doTask()
{
    gazebo_msgs::LinkState curr_state_;
    double angular_dist_;

    //we suppose task to be done, but if any of the agents uses setAngTwist task_done_ will be false
    task_done_ = true;
    //TO CHANGE FOR WITH ITERATOR
    //for(auto agent : this->group.getMembers())
    //{
        //check curretn angle of agent
        //ITERATOR : curr_state_ = this->getState(agent);

        //for quat to euler we need tf 1 not 2
        tf::Quaternion _q(
            curr_state_.pose.orientation.x,
            curr_state_.pose.orientation.y,
            curr_state_.pose.orientation.z,
            curr_state_.pose.orientation.w
        );
        
        curr_angle_ = this->quatToEuler(_q);

        //if we want a target, then agents will have a different cmd_angle_
        if(target_found_) this->computeAngle(curr_state_.pose.position.x, curr_state_.pose.position.y);

        angular_dist_ = cmd_angle_ - curr_angle_ ;
        if(abs(angular_dist_)<0.0001) ; //setAngle has already been used so do nothing
        //same here : else if(abs(angular_dist_) < 0.075) this->setAngle(agent);
        //else if(angular_dist_ < 0) this->setAngTwist(agent, -1);
        //else if(angular_dist_ > 0) this->setAngTwist(agent, +1);
    //}
}

void LookAtAction::setAngle(std::string agent)
{
    ros::spinOnce();
    
    final_cmd = this->getState(agent);

    //apply directly correct angle
    q_ = this->eulerToQuat(0., 0., this->cmd_angle_);
    final_cmd.pose.orientation.w = q_.w();
    final_cmd.pose.orientation.z = q_.z();
    final_cmd.pose.orientation.y = q_.y();
    final_cmd.pose.orientation.x = q_.x();

    final_cmd.twist.angular.x = 0;
    final_cmd.twist.angular.y = 0;
    final_cmd.twist.angular.z = 0;

    set_link_pub_.publish(final_cmd);
}

void LookAtAction::setAngTwist(std::string agent, int direction_)
{
    ros::spinOnce();
    
    final_cmd = this->getState(agent);

    if(abs(final_cmd.twist.angular.z) < 0.1) //no need to apply cmd if its already twisting
    {
        //apply ang twist
        final_cmd.twist.angular.z = direction_*ang_speed_;
        set_link_pub_.publish(final_cmd);
    }
    task_done_ = false; //important to execute once again the whole cycle
}

void LookAtAction::computeAngle(double x, double y)
{
    double d=sqrt(pow((target_x_-x),2)+pow((target_y_-y),2));
    if(d<0.01) {
        cmd_angle_ = curr_angle_; 
        ROS_ERROR("The target of lookataction may be a member of the group. This will cause weird behaviours.");
    }
    else
    {
        cmd_angle_ = atan2((target_y_-y),(target_x_-x));
    }
}

tf2::Quaternion LookAtAction::eulerToQuat(float r, float p, float y)
{
    tf2::Quaternion q;
    q.setRPY(r,p,y);
    q=q.normalize();
    return q;
}

double LookAtAction::quatToEuler(tf::Quaternion q)
{
    //careful : returns yaw only
    tf::Matrix3x3 m(q);
    double r, p, y;
    m.getRPY(r, p, y);

    return y;
}