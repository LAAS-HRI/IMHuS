#ifndef SOCIALSPACE
#define SOCIALSPACE

#include "socialPose.h"
#include "vector"
#include "tuple"
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "map_msgs/OccupancyGridUpdate.h"
#include "geometry_msgs/Pose.h"
#include "agents/entity.h"
#include <mutex>
#include "std_srvs/Empty.h"

// #include <boost/thread/recursive_mutex.hpp>

//SocialSpace is a location defined by X and Y. This place holds sub poses and 
//can assign them depending on an occupancy criteria. This class is typical for elevators
//bars, queues, bus, any place with a social placement

class SocialSpace 
{
    public:
        SocialSpace(std::string _id, agents::Entity* entities);

        void setX(float _x){x = _x;};
        void setY(float _y){y = _y;};
        void setID(std::string _id){id = _id;};
        float getX(){return x;};
        float getY(){return y;};
        std::string getID(){return id;};

        void add(SocialPose);
        Pose requestPose();

    private:
        void checkIfFree();
        Pose assignPose();

        std::string id;
        float x, y;        

        agents::Entity* entities;
        bool init=false;
        geometry_msgs::Pose o;
        float r, w, h;
        ros::Time last_check;

        std::vector<SocialPose> social_poses;

        std::mutex some_mutex;

        ros::NodeHandle nh_;
        ros::ServiceServer clearing;
        bool srvClear(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
        void Clear();
};

#endif