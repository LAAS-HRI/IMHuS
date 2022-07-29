#ifndef GOTO_ACT
#define GOTo_ACT

#include "actions/navigateAction.h"
#include "ros/ros.h"
#include <ros/package.h>
#include <string>
#include <vector>
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "std_srvs/Empty.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include "tf/tf.h"
#include <tf/transform_listener.h>
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"

// #include "map/pose.h"
#include "map/map.h"

namespace actions
{
    class GoToAction : public NavigateAction
    {
    public:
		// GoToAction();
		GoToAction(Pose);
		GoToAction(SocialSpace*);
        ~GoToAction(){};

		// void setPose(Pose);
		// Pose getPose(){return pose_;};

        virtual void execute();

    private:
		////////// METHODS //////////
		void init();
		void updateMarkerPose(float x, float y, float alpha);
		Pose getPose();
		
		////////// ATTRIBUTES //////////
		bool socialspace_;
		SocialSpace* socialspace_ptr_;
		Pose pose_;

		// Publishers //
		ros::Publisher pub_marker_rviz_;

		//// Variables ////
		ros::NodeHandle nh_;

		// Params
		visualization_msgs::Marker marker_rviz_;
        
    };
}


#endif