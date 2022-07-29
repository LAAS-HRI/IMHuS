#include "agents/state/conflictState.h"
#include "agents/state/goToState.h"
#include "agents/state/waitState.h"
#include "agents/state/idleState.h"

#define PLANNER "/imhus/move_base/GlobalPlanner/make_plan"
#define REPLAN_TIME 0.2

using namespace agents;

#define PI 3.1415
#define RATE 20.0

GoToState::GoToState(Entity *pEntity)
{
	entity_ = pEntity;
	ROS_INFO("%s is in GoToState.", entity_->getID().c_str());

	// Subscribers

	// // Publishers //pub should be in entity
	// cmd_pub_ = nh_.advertise<nav_msgs::Odometry>("/imhus/output/"+entity_->getID()+"/pref_vel", 10);

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
	//either we are replanning and we are too close to goal to have a valid plan,
	if (init_plan && distToGoal()<this->entity_->getRadius()) 
	{
		//this case is probably not used anymore
		// nav_msgs::Path final_path;
		// geometry_msgs::PoseStamped goal;
		// goal.pose.position.x = goal_pose_.getX();
		// goal.pose.position.y = goal_pose_.getY();
		// goal.pose.position.z = 0;
		// tf::Quaternion q;
		// q.setRPY(0,0,goal_pose_.getTheta());
		// goal.pose.orientation.w = q.w();
		// goal.pose.orientation.z = q.z();
		// goal.pose.orientation.y = q.y();
		// goal.pose.orientation.x = q.x();
		// goal.header.stamp = ros::Time::now();
		// goal.header.frame_id = "map";
		// final_path.poses.push_back(goal);

		// entity_->plan_pub_.publish(final_path);	
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
			ros::Duration(.5).sleep();
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
		
			this->sendPlan();
		}
	}
}

nav_msgs::GetPlan GoToState::requestPlan()
{
	//TODO: this mtd and replan() could probably be unified now
	//entity_->update();
	nav_msgs::GetPlan srv_;
	tf::Quaternion q_;
	tf::Quaternion q = entity_->getPose().getQuat();
	float x, y;
	float r = entity_->getRadius() + 0.1;
	float yaw = entity_->getPose().getTheta();

	x = entity_->getPose().getX();
	y = entity_->getPose().getY();
	// (no need for radius not used with orca)
	// x = entity_->getPose().getX() + r * cos(yaw);
	// y = entity_->getPose().getY() + r * sin(yaw);

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
	//failure for x reason : return empty, will be discarded
	return srv_;
}

void GoToState::sendPlan()
{
	// uncomment for sending plan directly without ORCA
	// entity_->sendPlan(plan);
	// last_plan_ = ros::Time::now();

	// ros::Duration(.5).sleep();
	// this->checkPlan();

	// for ORCA
	ros::Rate rate(RATE);
	geometry_msgs::Twist msg;
	geometry_msgs::PoseStamped next;
	ros::Time last_check_ = ros::Time::now();
	ros::Time last_replan_ = ros::Time::now();
	double r_, p_, yaw_;
	bool stuck=false;
	float prev_dist_goal=9999;
	int hard_blockage = 0, max_hb = 2;
	float goal_tolerance = 0.3;
	int lookup_pose=2;

	while(!stuck)
	{
		if(plan.poses.size() != 0)
		{
			this->cutPath();
			if(plan.poses.size()>lookup_pose+1)
			{
				next = plan.poses[lookup_pose];
				// plan.poses.erase(plan.poses.begin());

				// tf::Quaternion q_(next.pose.orientation.x, next.pose.orientation.y, next.pose.orientation.z, next.pose.orientation.w);
				// tf::Matrix3x3 m(q_);
				// m.getRPY(r_, p_, yaw_);

				msg.linear.x = (next.pose.position.x - this->entity_->getPose().getX())*RATE;
				msg.linear.y = (next.pose.position.y - this->entity_->getPose().getY())*RATE;
				// msg.angular.z = yaw_; //here we dont really send the twist, we just store the angle from the plan to apply it directly in the ORCA node
				this->entity_->cmd_pub_.publish(msg);

				if(ros::Time::now()> last_check_ +ros::Duration(2)) 
				{
					if(abs(this->distToGoal()-prev_dist_goal)<0.1) //detects when ORCA send static cmds
					{
						if(this->distToGoal()>=goal_tolerance && hard_blockage<=max_hb)
						{
							hard_blockage++;
							//just wait, maybe another agetn will clear the way
						}
						else if((this->distToGoal()>=goal_tolerance && hard_blockage>max_hb) || this->distToGoal()<goal_tolerance)
						{
							//these conditions are to detect when orca gets stuck near the goal when other agents are static like inside the elevator
							ROS_WARN("%s is probably stuck by other agents, it will stop here.", this->entity_->getID().c_str());
							stuck=true;
						}				
					} 
					last_check_ = ros::Time::now();
					prev_dist_goal = this->distToGoal();				
				}
				if(!stuck && ros::Time::now()> last_replan_ +ros::Duration(REPLAN_TIME))
				{
					last_replan_ = ros::Time::now();
					std::vector<geometry_msgs::PoseStamped> new_plan = this->replan();
					if(new_plan.size()!=0)
					{
						plan.poses = new_plan;
						plan.poses.erase(plan.poses.begin());
						plan.poses.erase(plan.poses.begin());
					}
				}
			}
			else
			{
				break;
			}
			
			rate.sleep();
		}
		else 
		{
			//verification at the end of execution to go on if we did nt reach the goal
			if(this->goalReached() || stuck) return;
			else
			{
				std::vector<geometry_msgs::PoseStamped> new_plan = this->replan();
				if(new_plan.size() != 0)
				{
					plan.poses = new_plan;					
				}
				else
				{
					ROS_WARN("%s may not be exactly at its goal but it will stop here because it cannot find a plan to continue.", this->entity_->getID().c_str());
					return;
				}
			}
		}
	}
}

std::vector<geometry_msgs::PoseStamped> GoToState::replan()
{
	nav_msgs::GetPlan srv_;
	tf::Quaternion q_;

	srv_.request.start.header.frame_id = "map";
	srv_.request.goal.header.frame_id = "map";
	srv_.request.start.pose.position.x = this->entity_->getPose().getX();
	srv_.request.start.pose.position.y = this->entity_->getPose().getY();
	srv_.request.start.pose.position.z = 0;
	q_.setRPY(0, 0, this->entity_->getPose().getTheta());
	srv_.request.start.pose.orientation.x = q_.x();
	srv_.request.start.pose.orientation.y = q_.y();
	srv_.request.start.pose.orientation.z = q_.z();
	srv_.request.start.pose.orientation.w = q_.w();

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
	if (distToGoal()<0.2)
		return true;
	return false;
}

void GoToState::jumpToGoal()
{

	// nav_msgs::Path final_path;
	// geometry_msgs::PoseStamped goal;
	// goal.pose.position.x = goal_pose_.getX();
	// goal.pose.position.y = goal_pose_.getY();
	// goal.pose.position.z = 0;
	// tf::Quaternion q;
	// q.setRPY(0,0,goal_pose_.getTheta());
	// goal.pose.orientation.w = q.w();
	// goal.pose.orientation.z = q.z();
	// goal.pose.orientation.y = q.y();
	// goal.pose.orientation.x = q.x();
	// goal.header.stamp = ros::Time::now();
	// goal.header.frame_id = "map";
	// final_path.poses.push_back(goal);

	// entity_->sendPlan(final_path);
	// ros::Duration(0.1).sleep();	
}

float GoToState::distToGoal()
{
	ros::spinOnce();
	return sqrt(pow(entity_->getPose().getX()-goal_pose_.getX(),2)+pow(entity_->getPose().getY()-goal_pose_.getY(),2));
}

void GoToState::cutPath()
{
	//this mtd should find the closest pose of the plan from the agent's pose
	if(plan.poses.size()>1)
	{
		float x=entity_->getPose().getX();
		float y=entity_->getPose().getY();

		float min_d = 9999, d;
		int min_i=0;
		for(int i=0; i<plan.poses.size(); i++)
		{
			d = sqrt(pow(plan.poses[i].pose.position.x-x,2)+pow(plan.poses[i].pose.position.y-y,2));
			if(d < min_d)
			{
				min_d = d;
				min_i = i;
			}
		}
		for(int i=0;i<min_i;i++) plan.poses.erase(plan.poses.begin());
	}
}