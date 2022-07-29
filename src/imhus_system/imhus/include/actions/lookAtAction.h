#include "turningAction.h"
#include "agents/group.h"
#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/LinkState.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/Quaternion.h> //for euler to quat
#include <tf/tf.h> //for quat to euler
#include <string>
#include <stdlib.h> //abs()
#include <math.h> 



//////////////////////////////////////////////////

//////////////////  Action /////////////////
namespace actions
{
    class LookAtAction : public TurningAction
    {
    public:
        LookAtAction();
        LookAtAction(float angle);
        LookAtAction(std::string target);
        ~ LookAtAction(){};

        virtual void execute();

    private:
        void findTarget();
        void linkStatesCB(gazebo_msgs::LinkStates);
        void init();
        void doTask();
        void setAngle(std::string);
        void setAngTwist(std::string, int);
        void computeAngle(double, double);
        gazebo_msgs::LinkState getState(std::string agent);
        tf2::Quaternion eulerToQuat(float, float, float);
        double quatToEuler(tf::Quaternion); //tf not tf2! returns yaw only

        
        bool task_done_;
        bool target_found_;
        double target_x_, target_y_;

        float cmd_angle_;
        float curr_angle_;
        float ang_speed_;
        agents::Group group_;
        std::string target_;

        //gazebo ros
        ros::NodeHandle nh_;

        ros::Publisher set_link_pub_;// = nh_.advertise<gazebo_msgs::LinkStates>("/gazebo/set_link_state", 10);
        ros::Subscriber link_states_sub_; // = nh_.subscribe("/gazebo/link_states", gazebo_msgs::LinkStates, &LookAtAction::linkStatesCB, this); 

        gazebo_msgs::LinkStates link_states;
        gazebo_msgs::LinkState final_cmd;

        tf2::Quaternion q_;
        ros::Time last_check_;
    };
}