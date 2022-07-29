#include <stdio.h>
#include <map>
#include <vector>
#include <math.h>

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/LinkState.h>
#include "gazebo_msgs/GetLinkState.h"
#include <ros/ros.h>
#include <std_msgs/UInt32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <ignition/math/Pose3.hh>
#include <sensor_msgs/JointState.h>
#include "std_srvs/Empty.h"


namespace gazebo
{
class GZ_PLUGIN_VISIBLE ElevatorPlugin : public ModelPlugin
{
    public:
    ElevatorPlugin();

    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    private:
        void OnUpdate();

        void requestCB(const std_msgs::String::ConstPtr &request);
        void jointStateCB(const sensor_msgs::JointState state);
        
        void executeRequest();
        void move();
        void closeDoor(int);
        void openDoor(int);
        void waitingDoor(int);
        void publishState();

        void initAgentsList(const gazebo_msgs::LinkStates::ConstPtr &msg);
        bool checkAgentsNearDoor(int door);
        void establishLinks(physics::ModelPtr _parent);
        void loadParameters(sdf::ElementPtr _sdf);

        ros::NodeHandle nh_;
        event::ConnectionPtr updateConnection;

        physics::ModelPtr model;
        physics::LinkPtr bodyLink;
        std::string modelName;

        ros::Subscriber door_event_sub, joint_state_sub, set_param_sub, gazebo_sub_;
        ros::Publisher door1_state_pub, door2_state_pub;
        ros::ServiceClient client;

        int waiting_time;
        double door_speed, fmax, joint1_pos, joint2_pos;

        bool agents_read = false;
        std::vector<std::string> agents_list_;

        enum DoorState{OPEN, OPENING, CLOSING, CLOSED};
        enum ElevatorState{FLOOR1, FLOOR2, MOVING};
        std::list<std::string> requestList;
        ElevatorState elevatorState;
        DoorState door1_state, door2_state;
        ros::Time prev_time, init_time_;


        //area of doors
        //p1 is the north-west point and then the others are clock wise
        const float door_size=2.5;
        const float p1y=-door_size/2, p2y=door_size/2;
        const float p4y=p1y, p3y=p2y;
        const float p1x=-0.2, p2x=-0.2, p3x=0.2, p4x=0.2;

        ros::ServiceClient clearing, clearing_d1, clearing_d2;
        std_srvs::Empty srvEmpty;

};

}