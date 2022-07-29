#ifndef PRUEBASC___GOTOSTATE_H
#define PRUEBASC___GOTOSTATE_H

#include "entityState.h"
#include "map/pose.h"
#include "map/socialSpace.h"
// #include "agents/entity.h"
//includes from gotoaction
#include "ros/ros.h"
#include <ros/package.h>
#include <string>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/String.h"
#include "move_base_msgs/MoveBaseGoal.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include <tf2/LinearMath/Quaternion.h>
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/GetPlan.h"
#include "navfn/MakeNavPlan.h"
#include "tf/transform_listener.h"

namespace agents
{

class GoToState : public EntityState
{
    public:
        GoToState(Entity* pEntity);
        ~GoToState(){};

        void goTo(Pose, SocialSpace*);
        void wait(float);
        void wait(std::string, std::string);
        bool acceptAsyncTask(){return false;};
        std::string getStateStr(){return "walking";};

    private:
        //methods of got action
        nav_msgs::GetPlan requestPlan();
        void askPlan(float agent_angle=-1);
        void sendPlan();
        void checkPlan();
        bool goalReached();
        void jumpToGoal();
        float distToGoal();

        std::vector<geometry_msgs::PoseStamped> replan(std::vector<geometry_msgs::PoseStamped>);
		
        //mtd for path comparison
        std::vector<geometry_msgs::PoseStamped> cutPath(std::vector<geometry_msgs::PoseStamped>);


        //ROS//
        float rate = 20.;
        ros::NodeHandle nh_;
        // // Publishers // (have been moved in entity.h for delay reasons)
        // Subscribers //
        //Services
        ros::ServiceClient make_plan_;

        // Params
        const float soft_conflict_wait_time_=10; ros::Time soft_conflict_start_time_;
        const float replan_freq_ = 2.;
        const int idx_replan_=10; //if too close to n_interpolate MB wont find any plan
        const int n_interpolate=5;
        int init_fails=0;
		bool going=false, pending_goto_=false;
        bool init_plan=false, soft_conflict=false;

		ros::Time last_check_, t, last_plan_;
        Pose goal_pose_, pending_pose_;
        SocialSpace* socialspace_ptr_;

        //
        // bool replan_=false;
        tf::TransformListener listener;
	    tf::StampedTransform transform;
        nav_msgs::Path plan;
        // navfn::MakeNavPlan srv_make_plan_;
        nav_msgs::GetPlan srv_make_plan_;
        nav_msgs::Odometry odom;
    
};

}


#endif 