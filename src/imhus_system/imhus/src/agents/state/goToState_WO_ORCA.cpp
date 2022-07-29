#include "agents/state/conflictState.h"
#include "agents/state/goToState.h"
#include "agents/state/waitState.h"
#include "agents/state/idleState.h"

#define PLANNER "/imhus/move_base/GlobalPlanner/make_plan"

using namespace agents;

#define PI 3.1415
#define RATE 20.0

GoToState::GoToState(Entity *pEntity)
{
	entity_ = pEntity;
	ROS_INFO("%s is in GoToState.", entity_->getID().c_str());

	// Subscribers

	// // Publishers //pub should be in entity

	// services
	ros::service::waitForService(PLANNER);
	make_plan_ = nh_.serviceClient<nav_msgs::GetPlan>(PLANNER, true);
}

void GoToState::goTo(Pose pose, SocialSpace* sp)
{
	socialspace_ptr_ = sp;
	if(!going)
	{
		going = true;
		goal_pose_ = pose;

		this->askPlan();
		if(!pending_goto_)
		{
			entity_->setState(new IdleState(entity_));
			delete this;
		}
		else
		{
			//here we go again with our goto from the scenario script 
			going = false;
			pending_goto_ = false;
			//init state
			init_plan=false;
			ROS_WARN("%s starts its 2nd goto action to :\n(%.1f,%.1f)", entity_->getID().c_str(),pending_pose_.getX(),pending_pose_.getY());
			this->goTo(pending_pose_, sp);
		}
	}
	else
	{
		//here an action got started when we were going somewhere because of a async action
		pending_pose_ = pose;
		pending_goto_ = true;
		
		ROS_WARN("A goto action has been started on %s while it was already going somewhere. \n It is probably because of a async action. It will be executed after the fisrt gotoaction.", entity_->getID().c_str());
	}
}

void GoToState::wait(float time)
{
	//TODO
	// what to do if im asked to wait while going somewhere ?
}
void GoToState::wait(std::string topic, std::string event)
{
	// what to do if im asked to wait while going somewhere ?
}

// void GoToState::go()
// {
// 	goal_status_.status = 1;
// 	last_check_ = ros::Time::now();
// 	while (goal_status_.status != 3)
// 	{
// 		// If replaning is active check if we can replan, 4 is when we failed to get a plan
// 		if (goal_status_.status == 4 && replan_active_)
// 		{
// 			ROS_WARN("Synchronization problems... Replanning... (this may means you tell an agent to go in an occupied pose by other agent.)");
// 			ros::Duration(1).sleep();
// 			this->askPlan();
// 		}
// 		else if (replans_ < max_replans_)
// 		{ // these replans are to make sure we use the correct tf transform because of latency
// 			this->askPlan();
// 			replans_++;
// 		}
// 		ros::spinOnce();
// 	}
// }

void GoToState::askPlan(float angle)
{
	last_plan_ = ros::Time::now();

	//either we are replanning and we are too close to goal to have a valid plan,
	if (init_plan && distToGoal()<this->entity_->getRadius()) 
	{
		nav_msgs::Path final_path;
		geometry_msgs::PoseStamped goal;
		goal.pose.position.x = goal_pose_.getX();
		goal.pose.position.y = goal_pose_.getY();
		goal.pose.position.z = 0;
		tf::Quaternion q;
		q.setRPY(0,0,goal_pose_.getTheta());
		goal.pose.orientation.w = q.w();
		goal.pose.orientation.z = q.z();
		goal.pose.orientation.y = q.y();
		goal.pose.orientation.x = q.x();
		goal.header.stamp = ros::Time::now();
		goal.header.frame_id = "map";
		final_path.poses.push_back(goal);

		entity_->plan_pub_.publish(final_path);	
		return;	
	}
	//either we were already at the goal and we skip action
	else if(this->goalReached())
	{
		ROS_INFO("%s already at goal pose, skipping goto action.", entity_->getID().c_str());
		return;
	}
	else
	{
		srv_make_plan_ = this->requestPlan();		
		
		if (srv_make_plan_.response.plan.poses.size() == 0)
		{
			init_fails++;
			ROS_ERROR("%s failed to get an initial plan. Will retry in 3 sec.", entity_->getID().c_str());
			ros::Duration(3).sleep();
			if(init_fails<4) 
				this->askPlan();
			else
			{
				entity_->setState(new ConflictState(entity_));
				entity_->goTo(goal_pose_, socialspace_ptr_);
				return;
			} 
		}
		else 
		{
			//received a valid plan			
			plan.poses = srv_make_plan_.response.plan.poses;
			init_plan = true;
			
			//add poses for smoothness
			geometry_msgs::PoseStamped p;
			float ratio;

			double r_, p_, yaw_;
			tf::Quaternion q_=entity_->getPose().getQuat();
			float yaw=entity_->getPose().getTheta();
			float x, y;
			float r = this->entity_->getRadius()+0.05;
			// tf::Matrix3x3 m(q_);
			// m.getRPY(r_, p_, yaw_);
			plan.poses[0].pose.orientation.x = q_.x();
			plan.poses[0].pose.orientation.y = q_.y();
			plan.poses[0].pose.orientation.z = q_.z();
			plan.poses[0].pose.orientation.w = q_.w();
			//normalize
			// if(yaw_<0) yaw_ = 6.28+yaw_;
			// if(yaw<0) yaw = 6.28+yaw;
			for(int i=1; i<n_interpolate+1; i++)
			{
				ratio = ((float)n_interpolate-(float)i)/(float)n_interpolate;
				p.pose.position.x = ratio*r*cos(yaw) + entity_->getPose().getX();
				p.pose.position.y = ratio*r*sin(yaw) + entity_->getPose().getY();
				// q_.setRPY(0,0,ratio*(yaw_-yaw)+yaw);
				// p.pose.orientation.x = q_.x();
				// p.pose.orientation.y = q_.y();
				// p.pose.orientation.z = q_.z();
				// p.pose.orientation.w = q_.w();
				
				p.pose.orientation = plan.poses[0].pose.orientation;
				plan.poses.insert(plan.poses.begin(), p);
			}
			this->sendPlan();
		}
	}
}

nav_msgs::GetPlan GoToState::requestPlan()
{
	//entity_->update();
	nav_msgs::GetPlan srv_;
	tf::Quaternion q_;
	tf::Quaternion q = entity_->getPose().getQuat();
	float x, y;
	float r = entity_->getRadius() + 0.1;
	float yaw = entity_->getPose().getTheta();

	x = entity_->getPose().getX() + r * cos(yaw);
	y = entity_->getPose().getY() + r * sin(yaw);

	//we make sure that we are far away from goal to avoid having red "failed to get a plan" msg on the terminal
	//but in any case the plan would be discarded later on even if we dont do this check
	if(distToGoal() > r)
	{
		srv_.request.start.header.frame_id = "map";
		srv_.request.goal.header.frame_id = "map";
		//x and y are taken outside our point cloud
		srv_.request.start.pose.position.x = x;
		srv_.request.start.pose.position.y = y;
		srv_.request.start.pose.position.z = 0;
		srv_.request.start.pose.orientation.x = q.x();
		srv_.request.start.pose.orientation.y = q.y();
		srv_.request.start.pose.orientation.z = q.z();
		srv_.request.start.pose.orientation.w = q.w();

		srv_.request.goal.pose.position.x = goal_pose_.getX();
		srv_.request.goal.pose.position.y = goal_pose_.getY();
		srv_.request.goal.pose.position.z = 0;

		// srv_make_plan_.request.tolerance = 2;
		q_.setRPY(0, 0, goal_pose_.getTheta());

		srv_.request.goal.pose.orientation.x = q_.x();
		srv_.request.goal.pose.orientation.y = q_.y();
		srv_.request.goal.pose.orientation.z = q_.z();
		srv_.request.goal.pose.orientation.w = q_.w();

		if (make_plan_.call(srv_)) 
			return srv_;
		
	}
	else
	{
		//here we send the goal directly since we are very close
		this->jumpToGoal();
	}
	//failure for x reason : return empty, will be discarded
	return srv_;
}

void GoToState::sendPlan()
{
	// uncomment for sending plan directly without ORCA
	entity_->sendPlan(plan);
	last_plan_ = ros::Time::now();

	ros::Duration(.5).sleep();
	this->checkPlan();

	// for ORCA
	// ros::Rate rate(RATE);
	// geometry_msgs::Twist msg;

	// for(auto next : plan.poses)
	// {
	// 	msg.linear.x = (next.pose.position.x - this->entity_->getPose().getX())*RATE;
	// 	msg.linear.y = (next.pose.position.y - this->entity_->getPose().getY())*RATE;
	// 	this->entity_->cmd_pub_.publish(msg);
	// 	rate.sleep();
	// }
	// if(!this->goalReached())
	// {
	// 	this->requestPlan();
	// }
}

void GoToState::checkPlan()
{
	ros::Rate r((int)replan_freq_);

	std::vector<geometry_msgs::PoseStamped> old_plan = cutPath(plan.poses);
	if(old_plan.size()<idx_replan_*2 || distToGoal()<this->entity_->getRadius()+0.1)
	{
		if(!soft_conflict) while(!goalReached()) ros::Duration(0.2).sleep(); //wait to reach goal and quit
		return; //means agent is not moving so quit the gotostate
	} 
	else 
	{
		std::vector<geometry_msgs::PoseStamped> new_plan = this->replan(old_plan);
		if(new_plan.size() != 0 && !soft_conflict)
		{
			//combine old and new pplans
			for(int i=0; i<=idx_replan_;i++) new_plan.insert(new_plan.begin(), old_plan[idx_replan_-i]);
			plan.poses = new_plan;
			this->sendPlan();
		}
		else if(new_plan.size() != 0 && soft_conflict)
		{
			//we were in conflict but it got solved
			soft_conflict = false;
			ROS_INFO("%s got out of the conflict, it will resume its navigation.", entity_->getID().c_str());
			plan.poses = new_plan;
			this->sendPlan();
		}
		else if(new_plan.size() == 0 && !soft_conflict && !this->goalReached())
		{
			//we should stop moving or slow down
			ROS_WARN("%s perceives a potential conflict, it will stop for a while to avoid collision !", entity_->getID().c_str());
			soft_conflict = true;
			nav_msgs::Path empty_;
			entity_->sendPlan(empty_);
			soft_conflict_start_time_ = ros::Time::now();

		}
		else if(new_plan.size() == 0 && soft_conflict && !this->goalReached() && ros::Time::now() > soft_conflict_start_time_+ros::Duration(soft_conflict_wait_time_))
		{
			entity_->setState(new ConflictState(entity_));
			entity_->goTo(goal_pose_, socialspace_ptr_);
			return;
		}

		r.sleep();
		if(!this->goalReached())
		{
			if(ros::Time::now()>last_plan_+ros::Duration(30))
			{
				ROS_ERROR("\n 30 secs have passed since begining of this action. \n %s failed to reach its goal !", entity_->getID().c_str());
				return;
			}
			else
			{
				this->checkPlan();			
			}	
		}
	}
}

std::vector<geometry_msgs::PoseStamped> GoToState::replan(std::vector<geometry_msgs::PoseStamped> plan)
{
	nav_msgs::GetPlan srv_;
	tf::Quaternion q_;

	srv_.request.start.header.frame_id = "map";
	srv_.request.goal.header.frame_id = "map";
	srv_.request.start.pose.position.x = plan[idx_replan_].pose.position.x;
	srv_.request.start.pose.position.y = plan[idx_replan_].pose.position.y;
	srv_.request.start.pose.position.z = 0;
	srv_.request.start.pose.orientation.x = plan[idx_replan_].pose.orientation.x;
	srv_.request.start.pose.orientation.y = plan[idx_replan_].pose.orientation.y;
	srv_.request.start.pose.orientation.z = plan[idx_replan_].pose.orientation.z;
	srv_.request.start.pose.orientation.w = plan[idx_replan_].pose.orientation.w;

	srv_.request.goal.pose.position.x = goal_pose_.getX();
	srv_.request.goal.pose.position.y = goal_pose_.getY();
	srv_.request.goal.pose.position.z = 0;

	// srv_make_plan_.request.tolerance = 2;
	q_.setRPY(0, 0, goal_pose_.getTheta());

	srv_.request.goal.pose.orientation.x = q_.x();
	srv_.request.goal.pose.orientation.y = q_.y();
	srv_.request.goal.pose.orientation.z = q_.z();
	srv_.request.goal.pose.orientation.w = q_.w();

	if (make_plan_.call(srv_)) 
	{
		return srv_.response.plan.poses;
	}
	else
	{
		ROS_ERROR("Cannot call srv for replanning !");
		return srv_.response.plan.poses;
	}
}

bool GoToState::goalReached()
{
	if (distToGoal()<0.01)
		return true;
	return false;
}

void GoToState::jumpToGoal()
{
	nav_msgs::Path final_path;
	geometry_msgs::PoseStamped goal;
	goal.pose.position.x = goal_pose_.getX();
	goal.pose.position.y = goal_pose_.getY();
	goal.pose.position.z = 0;
	tf::Quaternion q;
	q.setRPY(0,0,goal_pose_.getTheta());
	goal.pose.orientation.w = q.w();
	goal.pose.orientation.z = q.z();
	goal.pose.orientation.y = q.y();
	goal.pose.orientation.x = q.x();
	goal.header.stamp = ros::Time::now();
	goal.header.frame_id = "map";
	final_path.poses.push_back(goal);

	entity_->sendPlan(final_path);
	ros::Duration(0.1).sleep();	
}

float GoToState::distToGoal()
{
	ros::spinOnce();
	return sqrt(pow(entity_->getPose().getX()-goal_pose_.getX(),2)+pow(entity_->getPose().getY()-goal_pose_.getY(),2));
}

std::vector<geometry_msgs::PoseStamped> GoToState::cutPath(std::vector<geometry_msgs::PoseStamped> v)
{
	float x=entity_->getPose().getX();
	float y=entity_->getPose().getY();

	float min_d = 9999, d;
	int min_i=0;
	for(int i=0; i<v.size(); i++)
	{
		d = sqrt(pow(v[i].pose.position.x-x,2)+pow(v[i].pose.position.y-y,2));
		if(d < min_d)
		{
			min_d = d;
			min_i = i;
		}
	}
	for(int i=0;i<min_i;i++) v.erase(v.begin());
	if(v.size()==0) ROS_ERROR("%s : could not cut path, ignore checking", entity_->getID().c_str());
	return v;
}
