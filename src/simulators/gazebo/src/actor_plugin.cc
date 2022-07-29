#include "../include/actor_plugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(ActorPlugin)

#define RATE 50.0
#define WALKING_ANIMATION "walking"
#define STANDING_ANIMATION "standing"

/////////////////////////////////////////////////
ActorPlugin::ActorPlugin()
{ 
//   int argc = 0 ;
//   ros::init(argc, NULL, "", ros::init_options::AnonymousName);
}

/////////////////////////////////////////////////
void ActorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->sdf = _sdf;
  this->model = _model;
  this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
  this->name = this->model->GetName();
  // this->link =  model->GetLink(name+"::"+name+"_pose");
  ROS_INFO("ActorPlugin says : A human has name %s", name.c_str());
  tf2_ros::TransformBroadcaster tf_br_;

  if (_sdf->HasElement("animation_factor"))
    this->animationFactor = _sdf->Get<double>("animation_factor");
  else
    this->animationFactor = 4.5;

  //Subscribers
  cmd_pose_sub_ = nh_.subscribe<geometry_msgs::Pose>("/orca/output/"+name+"/cmd_pose", 10, &ActorPlugin::CmdPoseCB, this);
  cmd_twist_sub_ = nh_.subscribe<geometry_msgs::Twist>("/orca/output/"+name+"/cmd_twist", 10, &ActorPlugin::CmdTwistCB, this);
  plan_sub_ = nh_.subscribe<nav_msgs::Path>("/imhus/output/"+name+"/plan", 10, &ActorPlugin::PlanCB, this);
  reset_sub_ = nh_.subscribe<std_msgs::String>("/gazebo/reset", 10, &ActorPlugin::resetCB, this);

  //Publishers
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/imhus/input/"+name+"/odom", 10);

  //INIT POSES ! (only time when it should be done)
  real_pose = model->WorldPose();
  init_pose_ = real_pose;
  // anim_pose = real_pose;
  anim_pose.Set(real_pose.Pos().X(), real_pose.Pos().Y(), actor_height,real_pose.Rot().Roll()+1.57, 0, real_pose.Rot().Yaw()+1.57);

  // Create custom trajectory
  this->trajectoryInfo.reset(new physics::TrajectoryInfo());
  this->trajectoryInfo->type = STANDING_ANIMATION;

  this->actor->SetCustomTrajectory(this->trajectoryInfo);
  this->actor->SetWorldPose(anim_pose, false, false);
  this->actor->SetScriptTime(this->actor->ScriptTime() + (0.1 * this->animationFactor));  
  last_check_ = ros::Time::now();

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          std::bind(&ActorPlugin::OnUpdate, this, std::placeholders::_1)));

}

/////////////////////////////////////////////////
void ActorPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  //gazebo turns at least at 1000hz, this dont need to be that fast
  if(ros::Time::now()>last_spin_ + ros::Duration(1.0/RATE)) 
  {
    ros::spinOnce();
    // this->followPlan();
    this->updatePose();
    last_spin_ = ros::Time::now();
  }
}

void ActorPlugin::CmdTwistCB(geometry_msgs::Twist msg)
{
  this->twist = msg;
  if(sqrt(pow(this->twist.linear.x,2)+pow(this->twist.linear.y,2))>0.1)
  {
    is_moving = true;
    last_move_ = ros::Time::now(); //uncomment if we command agent by twist
  }
  
  // Since commands by twist are not well supported for actors in Gazebo, we dont use this cmd
  // the purpose of storing the agent's twist is just to publish it in the odometry in the odom_pub_ topic as it can be useful elsewhere
  // indeed it is used by Cohan to predict the agent's paths
}

void ActorPlugin::CmdPoseCB(geometry_msgs::Pose msg)
{
  //used with ORCA
  // is_moving = true;
  // last_move_ = ros::Time::now();
    
  dt = ros::Duration(ros::Time::now()-last_check_).toSec();
  last_check_ = ros::Time::now();

  ignition::math::Pose3d pose{msg.position.x, msg.position.y, actor_height, msg.orientation.w, msg.orientation.x,msg.orientation.y,msg.orientation.z}; 

  anim_pose.Set(pose.Pos().X(),pose.Pos().Y(),actor_height,pose.Rot().Roll()+1.57,0,pose.Rot().Yaw()+1.57);
  this->actor->SetWorldPose(anim_pose, false, false);
  this->actor->SetScriptTime(this->actor->ScriptTime() + (dt * this->animationFactor)); 
}

void ActorPlugin::followPlan()
{
  //not used if ORCA is on
  geometry_msgs::PoseStamped p;
  if(ros::Time::now() > last_check_ + ros::Duration(0.05) && my_plan.poses.size() != 0)
  {
    p = my_plan.poses[0];
    my_plan.poses.erase(my_plan.poses.begin()); //like a popfront

    //stuff for the animation
    last_move_ = ros::Time::now();
    
    dt = ros::Duration(ros::Time::now()-last_check_).toSec();
    last_check_ = ros::Time::now();

    ignition::math::Pose3d pose{p.pose.position.x, p.pose.position.y, actor_height, p.pose.orientation.w, p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z}; 

    anim_pose.Set(pose.Pos().X(),pose.Pos().Y(),actor_height,pose.Rot().Roll()+1.57,0,pose.Rot().Yaw()+1.57);
    this->actor->SetWorldPose(anim_pose, false, false);
    this->actor->SetScriptTime(this->actor->ScriptTime() + (dt * this->animationFactor)); 
  }
}

void ActorPlugin::PlanCB(nav_msgs::Path p)
{
  is_moving = true;
  my_plan = p;
}

void ActorPlugin::updatePose()
{
  anim_pose = this->model->WorldPose();
  real_pose.Set(anim_pose.Pos().X(),anim_pose.Pos().Y(), 0, anim_pose.Rot().Roll()-1.57,anim_pose.Rot().Pitch(),anim_pose.Rot().Yaw()-1.57);

  this->publishOdom();
  this->updateAnim();
}

void ActorPlugin::updateAnim()
{
  if(last_move_ + ros::Duration(0.1) < ros::Time::now() && this->trajectoryInfo->type == WALKING_ANIMATION)
  { 
    // this->trajectoryInfo.reset(new physics::TrajectoryInfo());
    this->trajectoryInfo->type = STANDING_ANIMATION;
    this->actor->SetCustomTrajectory(this->trajectoryInfo);
    is_moving = false;
    this->twist.linear.x = 0;
    this->twist.linear.y = 0;
    this->twist.angular.z = 0;
  } 
  else if( is_moving && this->trajectoryInfo->type == STANDING_ANIMATION)
  {
    // this->trajectoryInfo.reset(new physics::TrajectoryInfo());
    this->trajectoryInfo->type = WALKING_ANIMATION;
    this->actor->SetCustomTrajectory(this->trajectoryInfo);
  }
}

void ActorPlugin::publishOdom()
{
  odom.pose.pose.position.x = real_pose.Pos().X();
  odom.pose.pose.position.y = real_pose.Pos().Y();
  odom.pose.pose.position.z = real_pose.Pos().Z();
  odom.pose.pose.orientation.x = real_pose.Rot().X();
  odom.pose.pose.orientation.y = real_pose.Rot().Y();
  odom.pose.pose.orientation.z = real_pose.Rot().Z();
  odom.pose.pose.orientation.w = real_pose.Rot().W();
  //need for twist ? yes for cohan
  odom.twist.twist.linear.x = this->twist.linear.x;
  odom.twist.twist.linear.y = this->twist.linear.y;
  odom.twist.twist.angular.z = this->twist.angular.z;
  odom_pub_.publish(odom);
}

void ActorPlugin::resetCB(std_msgs::String s)
{
  //reset agent init pose
  geometry_msgs::Pose pose;
  pose.position.x = init_pose_.Pos().X();
  pose.position.y = init_pose_.Pos().Y();
  pose.orientation.x = init_pose_.Rot().X();
  pose.orientation.y = init_pose_.Rot().Y();
  pose.orientation.z = init_pose_.Rot().Z();
  pose.orientation.w = init_pose_.Rot().W();
  this->CmdPoseCB(pose);
  this->twist.linear.x = 0;
  this->twist.linear.y = 0;
  this->twist.angular.z = 0;
}