#include "agents/state/conflictState.h"
#include "agents/state/goToState.h"
#include "agents/state/waitState.h"
#include "agents/state/idleState.h"

using namespace agents;

#define PI 3.1415

GoToState::GoToState(Entity *pEntity)
{
	entity_ = pEntity;
	ROS_INFO("%s is in GoToState.", entity_->getID().c_str());

	// Subscribers

	// // Publishers //pub should be in entity

	// services
	ros::service::waitForService("/human1/move_base/GlobalPlanner/make_plan");
	make_plan_ = nh_.serviceClient<nav_msgs::GetPlan>("/human1/move_base/GlobalPlanner/make_plan", true);
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
	// PREVIOUS WAY OF DOING THINGS////////////////////////////////////////////////////
	//  move_base_msgs::MoveBaseGoal move_base_goal = this->getMoveBaseGoal(pose_);
	//
	//  move_base_msgs::MoveBaseActionGoal nav_goal;
	//  nav_goal.goal.target_pose.header.frame_id = "map";
	//  nav_goal.goal.target_pose.header.stamp = ros::Time::now();
	//  nav_goal.goal = move_base_goal;
	//  this->entity_->pub_goal_move_base_.publish(nav_goal);
	//  ros::Duration(0.03).sleep();
	///////////////////////////////////////////////////////////////////////////////////
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
				// ROS_ERROR("%s cannot find a initial plan, it will be teleported to its goal !", entity_->getID().c_str());
				// this->jumpToGoal();
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
			conflict = false;
			
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
	float r = entity_->getRadius() + 0.05;
	float yaw = entity_->getPose().getTheta();

	x = entity_->getPose().getX() + r * cos(yaw);
	y = entity_->getPose().getY() + r * sin(yaw);

	//we make sure that we are far away from goal to avoid having red "failed to get a plan" msg on the terminal
	//but in any case the plan would be discarded later on even if we dont do this check
	if(distToGoal() > r)
	{
		// ros::Time t1=ros::Time::now();
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
		{
			// ros::Time t2=ros::Time::now();
			// ROS_WARN("duration : %.6f ", (t2-t1).toSec());
			return srv_;
		}
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
	entity_->sendPlan(plan);
	last_plan_ = ros::Time::now();

	ros::Duration(.5).sleep();
	// nb_checks = 0;
	this->checkPlan();
}

void GoToState::checkPlan()
{
	ros::Rate r((int)replan_freq_);
	nb_checks++;
	bool plan_changed=false;
	nav_msgs::GetPlan new_plan_full = this->requestPlan();
	std::vector<geometry_msgs::PoseStamped> new_plan = new_plan_full.response.plan.poses;

	if(new_plan.size() != 0 && !conflict)
	{
		std::vector<geometry_msgs::PoseStamped> old_plan = cutPath(plan.poses);
		if(old_plan.size()==0) checkPlan(); //sometimes the poses are not matching, we shoud ignore this case
		// float mse = MSE(new_plan, old_plan);
		
		//
		//entity_->update();
		//check for change in path
		float lo = computePathLength(old_plan);
		float ln = computePathLength(new_plan);
		ln += entity_->getRadius();
	
		// if(mse > MSE_tolerance) plan_changed = true;
		if(abs((lo-ln)/lo>0.05) && lo>1)
		{
			//this condition is to ignore the changes that are always detected at the end when the path is less than 1
			plan_changed = true;
		}

		if(plan_changed && nb_checks>2)
		{
			ROS_WARN("%s : %.2f", entity_->getID().c_str(), abs(lo-ln)/lo);
			ROS_WARN("%s : %.3f and %.3f", entity_->getID().c_str(), lo,ln);
			//we discard the first one because it is triggered all the time due to our initial pose trick to call the planner
			ROS_WARN("%s's plan got changed !", entity_->getID().c_str());
			plan.poses = new_plan;
			this->sendPlan();
		}
	}
	else if(new_plan.size() != 0 && conflict)
	{
		//we were in conflict but it got solved
		conflict = false;
		ROS_INFO("%s got out of the conflict, it will resume its navigation.", entity_->getID().c_str());
		plan.poses = new_plan;
		this->sendPlan();
	}
	else if(new_plan.size() == 0 && !conflict && !this->goalReached())
	{
		std::vector<geometry_msgs::PoseStamped> old_plan = cutPath(plan.poses);
		float lo = computePathLength(old_plan);
		//we should stop moving or slow down
		if(lo<3)
		{
			ROS_WARN("%s perceives a potential conflict, it will stop for a while to avoid collision !", entity_->getID().c_str());
			conflict = true;
			nav_msgs::Path empty_;
			entity_->sendPlan(empty_);
			soft_conflict_start_time_ = ros::Time::now();
		}
		else
		{

		}
	}
	else if(new_plan.size() == 0 && conflict && !this->goalReached() && ros::Time::now() > soft_conflict_start_time_+ros::Duration(soft_conflict_wait_time_))
	{
		if(socialspace_ptr_!=nullptr)
		{
			//if we are here, it means we wait for some time in the case just above, but there is still a conflict, so we should act and not just wait
			entity_->setState(new ConflictState(entity_));
			entity_->goTo(goal_pose_, socialspace_ptr_);
			return;
		}
		else
		{
			ROS_WARN("%s is trying to modify its goal to avoid a conflict.",entity_->getID().c_str());
			this->autoModifyGoal();
		}
	}

	r.sleep();
	if(!this->goalReached())
	{
		if(ros::Time::now()>last_plan_+ros::Duration(30))
		{
			ROS_ERROR("\n 30 secs have passed since begining of this action. \n %s failed to reach its goal on its own!", entity_->getID().c_str());
			jumpToGoal();
		}
		else
		{
			// ros::Duration(check_plan_time).sleep();
			if(distToGoal() > 2*entity_->getRadius()) this->checkPlan();
			else while(!goalReached()){};
			
		}	
	}
}

void GoToState::autoModifyGoal()
{
	if(plan.poses.size()>21)
	{
		for(int i=0;i<20;i++)
			plan.poses.pop_back();
		
		goal_pose_.setX(plan.poses.back().pose.position.x);
		goal_pose_.setY(plan.poses.back().pose.position.y);
		this->checkPlan();
	}
	else if(plan.poses.size()>1)
	{
		for(int i=0;i<plan.poses.size()-1;i++)
			plan.poses.pop_back();
		
		goal_pose_.setX(plan.poses.back().pose.position.x);
		goal_pose_.setY(plan.poses.back().pose.position.y);
		this->checkPlan();
	}
	else
	{
		ROS_ERROR("%s could not find another satisfying goal.", entity_->getID().c_str());
		this->jumpToGoal();
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
	return sqrt(pow(entity_->getPose().getX()-goal_pose_.getX(),2)+pow(entity_->getPose().getY()-goal_pose_.getY(),2));
}

std::vector<geometry_msgs::PoseStamped> GoToState::cutPath(std::vector<geometry_msgs::PoseStamped> v)
{
	//from time that have passed
	// float n = (ros::Time::now()-last_plan_).toSec()*rate;
	// int i_max = static_cast<int>(n);

	// if(!i_max>v.size()) i_max = v.size()-1;
	
	// for(int i=0; i<i_max;i++)
	// {
	// 	v.erase(v.begin()); //like a popfront
	// }
	
	float x=entity_->getPose().getX();
	float y=entity_->getPose().getY();
	// for(auto p : v)
	// {
	// 	if(sqrt(pow(p.pose.position.x-x,2)+pow(p.pose.position.y-y,2))<0.05) 
	// 	{
	// 		// ROS_WARN("pose found");
	// 		break;
	// 	}
	// 	v.erase(v.begin());
	// }
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

float GoToState::computePathLength(std::vector<geometry_msgs::PoseStamped> v)
{
	geometry_msgs::Point p, pp;

	float sum=0;
	for(int i=1; i<v.size(); i++)
	{
		p = v[i].pose.position;
		pp = v[i-1].pose.position;
		sum += sqrt(pow(p.x-pp.x,2)+pow(p.y-pp.y,2));
	}
	return sum;
}

float GoToState::MSE(std::vector<geometry_msgs::PoseStamped> new_v, std::vector<geometry_msgs::PoseStamped> old_v)
{
	float sum=0;

	int i_max;
	if(new_v.size() > look_ahead && old_v.size() > look_ahead+n_interpolate) i_max = look_ahead;
	else i_max = std::min(new_v.size(), old_v.size())-n_interpolate-1;

	if(i_max>0)
	{
		for(int i=0; i<i_max; i++)
		{
			sum += pow(new_v[i].pose.position.x-old_v[i+n_interpolate].pose.position.x,2)+pow(new_v[i].pose.position.y-old_v[i+n_interpolate].pose.position.y,2);
		}

		return sum/(float)i_max;
	}
	else
	{
		return 0;
	}
}