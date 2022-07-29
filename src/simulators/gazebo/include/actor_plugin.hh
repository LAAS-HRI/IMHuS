/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef GAZEBO_PLUGINS_ACTORPLUGIN_HH_
#define GAZEBO_PLUGINS_ACTORPLUGIN_HH_

#include <string>
#include <vector>
#include <functional>
#include <ignition/math.hh>
#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/util/system.hh"
#include "gazebo_msgs/LinkStates.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/String.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "visualization_msgs/MarkerArray.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf/LinearMath/Quaternion.h>
#include "nav_msgs/Odometry.h"
#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/TransformStamped.h>
#include "tf/tf.h"
#include "nav_msgs/Path.h"
#include "thread"


namespace gazebo
{
  class GZ_PLUGIN_VISIBLE ActorPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: ActorPlugin();

    /// \brief Load the actor plugin.
    /// \param[in] _model Pointer to the parent model.
    /// \param[in] _sdf Pointer to the plugin's SDF elements.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

   private: 
    void OnUpdate(const common::UpdateInfo &_info);
    void CmdPoseCB(geometry_msgs::Pose);
    void CmdTwistCB(geometry_msgs::Twist);
    void PlanCB(nav_msgs::Path);
    void followPlan();
    void updatePose();
    void updateAnim();
    void publishOdom();
    void resetCB(std_msgs::String useless);

    /// \brief List of connections
    private: std::vector<event::ConnectionPtr> connections;
    /// \brief Pointer to the parent actor.
    private: physics::ActorPtr actor;

    /// \brief Pointer to the world, for convenience.
    private: physics::WorldPtr world;
    physics::ModelPtr model;
    physics::LinkPtr link;

    /// \brief Pointer to the sdf element.
    private: sdf::ElementPtr sdf;

    /// \brief Time scaling factor. Used to coordinate translational motion
    /// with the actor's walking animation.
    private: double animationFactor = 1.0;

    ignition::math::Pose3d init_pose_; //only for reinitialisation from the user
    ignition::math::Pose3d real_pose, anim_pose;
    geometry_msgs::Twist twist;

    float vx,vy,wz, goalx, goaly, goalqz, goalqw, dt;
    const float goal_tolerance=0.2, actor_height=1.00, rate=100;

    ros::NodeHandle nh_;
    std::string name;
    ros::Subscriber reset_sub_; 
    ros::Subscriber cmd_pose_sub_;
    ros::Subscriber cmd_twist_sub_;
    ros::Subscriber plan_sub_;
    // ros::Subscriber odom_sub_; //this one is just to broadcast the TFs, it is required to do like this..
    ros::Publisher odom_pub_;
    nav_msgs::Odometry odom;

    nav_msgs::Path my_plan;

    tf2_ros::TransformBroadcaster tf_br_;
    geometry_msgs::TransformStamped unit_transform, transform;

    ros::Time last_check_, last_move_=ros::Time::now(), last_spin_=ros::Time::now();
    bool is_moving=false, new_plan_received=false;

    physics::TrajectoryInfoPtr trajectoryInfo;

    std::vector<geometry_msgs::Pose>* plan;

  };
}
#endif
