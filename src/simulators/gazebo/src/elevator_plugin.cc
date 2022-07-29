// This node controls an elevator in gazebo
#include "elevator_plugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(ElevatorPlugin)

ElevatorPlugin::ElevatorPlugin()
{
  clearing = nh_.serviceClient<std_srvs::Empty>("/elevator/clear");
  clearing_d1 = nh_.serviceClient<std_srvs::Empty>("/elevator_door1/clear");
  clearing_d2 = nh_.serviceClient<std_srvs::Empty>("/elevator_door2/clear");
  init_time_ = ros::Time::now();
}

void ElevatorPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  establishLinks(_parent);
  loadParameters(_sdf);

  model->GetJoint("door1_joint")->SetParam("fmax", 0, fmax);
  model->GetJoint("door2_joint")->SetParam("fmax", 0, fmax);

  elevatorState = FLOOR1;
  door1_state = CLOSED;
  door2_state = CLOSED;
  joint1_pos = 0;
  joint2_pos = 0;

}

void ElevatorPlugin::OnUpdate()
{
  ros::spinOnce();
  publishState();
}

void ElevatorPlugin::loadParameters(sdf::ElementPtr _sdf)
{
  if (_sdf->HasElement("door_wait_time"))
  {
    waiting_time = _sdf->GetElement("door_wait_time")->Get<int>();
  }
  else
  {
    waiting_time = 10;
  }
  if (_sdf->HasElement("door_speed"))
  {
    door_speed = _sdf->GetElement("door_speed")->Get<double>();
  }
  else
  {
    door_speed = 0.5;
  }
  if (_sdf->HasElement("door_speed"))
  {
    fmax = _sdf->GetElement("fmax")->Get<double>();
  }
  else
  {
    fmax = 10.0;
  }
  // ROS_WARN("Plugin loaded successfully and has read the parameters.");
}

void ElevatorPlugin::establishLinks(physics::ModelPtr _parent)
{
  model = _parent;
  bodyLink = model->GetLink("link"); // unused
  modelName = model->GetName();      // unused

  // Subscribers
  door_event_sub = nh_.subscribe<std_msgs::String>("/elevator/request", 100, &ElevatorPlugin::requestCB, this);
  joint_state_sub = nh_.subscribe<sensor_msgs::JointState>("/elevator/joint_states", 100, &ElevatorPlugin::jointStateCB, this);
  gazebo_sub_ = nh_.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 100, &ElevatorPlugin::initAgentsList, this);

  // Publishers
  door1_state_pub = nh_.advertise<std_msgs::String>("/elevator/door1_state", 100);
  door2_state_pub = nh_.advertise<std_msgs::String>("/elevator/door2_state", 100);

  //services
  client = nh_.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");

  updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ElevatorPlugin::OnUpdate, this));
}

void ElevatorPlugin::initAgentsList(const gazebo_msgs::LinkStates::ConstPtr& msg)
{
  if(!agents_read && ros::Time::now() > init_time_+ros::Duration(2)) //INIT (we wait for a bit of time because tiago takes time to arrive in sim)
  {
    for(std::string link : msg->name)
    {
      std::size_t f1 = link.find("human"), f2 = link.find("_pose");
      if (f1!=std::string::npos && f2!=std::string::npos)
      {
        agents_list_.push_back(link);
      }
      //tiago
      if(link == "tiago::base_footprint")  agents_list_.push_back(link);
    }
    agents_read = true;
    //print it to make sure they are all read (to avoid doors closing on them)
    ROS_INFO("Agents' frames known by elevator plugin :");
    for(std::string agent: agents_list_)
    {
      ROS_INFO("%s", agent.c_str());
    }
  }
}

void ElevatorPlugin::requestCB(const std_msgs::String::ConstPtr &request)
{
  if (request->data == "door1")
    requestList.push_back(request->data);
  else if (request->data == "door2")
    requestList.push_back(request->data);
  else if (request->data == "floor1")
    requestList.push_back(request->data);
  else if (request->data == "floor2")
    requestList.push_back(request->data);
  else
    ROS_WARN("Elevator request unknown.");

  executeRequest();
}

void ElevatorPlugin::executeRequest()
{
  if (!requestList.empty())
  {
    if (requestList.front() == "door1")
    {
      if (elevatorState == FLOOR1)
      {
        openDoor(1);
        requestList.pop_front();
      }
      else if (elevatorState == FLOOR2)
      {
        closeDoor(2);
        move();
        executeRequest(); // or openDoor its equivalent here
      }
    }
    else if (requestList.front() == "door2")
    {
      if (elevatorState == FLOOR2)
      {
        openDoor(2);
        requestList.pop_front();
      }
      else if (elevatorState == FLOOR1)
      {
        closeDoor(1);
        move();
        executeRequest();
      }
    }
    else if (requestList.front() == "floor1")
    {
      if (elevatorState == FLOOR2)
      {
        closeDoor(2);
        move();
        executeRequest();
      }
      else if (elevatorState == FLOOR1)
      {
        openDoor(1);
        requestList.pop_front();
        ROS_INFO("Elevator arrived at floor 1.");
      }
    }
    else if (requestList.front() == "floor2")
    {
      if (elevatorState == FLOOR1)
      {
        closeDoor(1);
        move();
        executeRequest();
      }
      else if (elevatorState == FLOOR2)
      {
        openDoor(2);
        requestList.pop_front();
        ROS_INFO("Elevator arrived at floor 2.");
      }
    }
    else
    {
      ROS_WARN("Request not understood. Ingoring it.");
      requestList.pop_front();
    }
  }
}

bool ElevatorPlugin::checkAgentsNearDoor(int door)
{
  gazebo_msgs::GetLinkState get_link_state;
  float x, y;

  get_link_state.request.reference_frame = "map";
  for(std::string agent : agents_list_)
  {
    get_link_state.request.link_name = agent;
    if(client.call(get_link_state))
    {
      x = get_link_state.response.link_state.pose.position.x;
      y = get_link_state.response.link_state.pose.position.y;
      // ROS_WARN("%s %f %f", agent.c_str(), x, y);
      if(door == 1)
      {
        if(x > door_size/2-0.2 && x < door_size/2+0.2 && y > p4y && y < p3y)return true;
      }
      else if(door==2)
      {
        if(x > -door_size/2-0.2 && x < -door_size/2+0.2 && y > p4y && y < p3y) return true;
      }
    }
  }
  return false;
}

void ElevatorPlugin::move()
{
  //this simulates the elevator changing floors
  if (elevatorState == FLOOR1)
  {
    ROS_INFO("elevator going to floor 2...");
    elevatorState = MOVING;
    ros::Duration(3).sleep();
    elevatorState = FLOOR2;
    ros::Duration(1).sleep();
  }
  else if (elevatorState == FLOOR2)
  {
    ROS_INFO("elevator going to floor 1...");
    elevatorState = MOVING;
    ros::Duration(3).sleep();
    elevatorState = FLOOR1;
    ros::Duration(1).sleep();
  }
}

void ElevatorPlugin::jointStateCB(const sensor_msgs::JointState state)
{
  if (state.name[0] == "door1_joint")
    joint1_pos = state.position[0];
  if (state.name[0] == "door2_joint")
    joint2_pos = state.position[0];
}

void ElevatorPlugin::closeDoor(const int door)
{
  bool crossing = false;
  if (door == 1)
  {
    model->GetJoint("door1_joint")->SetParam("vel", 0, -door_speed);

    while (joint1_pos > 0.001 && !crossing)
    {
      crossing = this->checkAgentsNearDoor(1);
      door1_state = CLOSING;
      ros::Duration(0.1).sleep();
    }
    if(crossing)
    {
        this->openDoor(1);
    } else{
       model->GetJoint("door1_joint")->SetParam("vel", 0, 0.0);
      door1_state = CLOSED;
    }   
  }
  else if (door == 2)
  {
    model->GetJoint("door2_joint")->SetParam("vel", 0, -door_speed);

    while (joint2_pos > 0.001 && !crossing)
    {
      crossing = this->checkAgentsNearDoor(2);
      door2_state = CLOSING;
      ros::Duration(0.1).sleep();
    }

    if(crossing)
    { 
      this->openDoor(2);
    }else{
      model->GetJoint("door2_joint")->SetParam("vel", 0, 0.0);
      door2_state = CLOSED;
    }
  }
  if(!clearing.call(srvEmpty)) ROS_WARN("Could not clear the socialspace elevator.");
  if(!clearing_d1.call(srvEmpty)) ROS_WARN("Could not clear the socialspace elevator_door1.");
  if(!clearing_d2.call(srvEmpty)) ROS_WARN("Could not clear the socialspace elevator_door2.");
}

void ElevatorPlugin::openDoor(const int door)
{
  if (door == 1)
  {
    model->GetJoint("door1_joint")->SetParam("vel", 0, door_speed);

    while (joint1_pos < door_size - 0.001)
    {
      door1_state = OPENING;
    }

    // stopping the door
    model->GetJoint("door1_joint")->SetParam("vel", 0, 0.0);

    door1_state = OPEN;
    ROS_INFO("Door 1 is open.");

    // maintain open doors
    waitingDoor(1);
  }
  else if (door == 2)
  {
    model->GetJoint("door2_joint")->SetParam("vel", 0, door_speed);

    while (joint2_pos < door_size - 0.001)
    {
      door2_state = OPENING;
    }

    // stopping the door
    model->GetJoint("door2_joint")->SetParam("vel", 0, 0.0);

    door2_state = OPEN;
    ROS_INFO("Door 2 is open.");
    // maintain open doors
    waitingDoor(2);
  }
}

void ElevatorPlugin::waitingDoor(const int door_id)
{
  // we wait for 10 sec with open door
  ros::Duration(waiting_time).sleep();
  if (door_id == 1)
  {
    closeDoor(1);
  }
  else if (door_id == 2)
  {
    closeDoor(2);
  }
}

void ElevatorPlugin::publishState()
{
  // we publish at a slow rate the doors' states
  ros::Time current;
  current = ros::Time::now();

  if (current > prev_time + ros::Duration(0.25))
  {
    std_msgs::String msg;
    std::string d1 = "closed", d2 = "closed";

    // convert to a string
    if (door1_state == OPEN)
      d1 = "open";
    if (door2_state == OPEN)
      d2 = "open";
    if (door1_state == OPENING)
      d1 = "opening";
    if (door2_state == OPENING)
      d2 = "opening";
    if (door1_state == CLOSING)
      d1 = "closing";
    if (door2_state == CLOSING)
      d2 = "closing";

    // publish
    msg.data = d1;
    door1_state_pub.publish(msg);

    msg.data = d2;
    door2_state_pub.publish(msg);

    // change last time of publication
    prev_time = ros::Time::now();
  }

  
}
