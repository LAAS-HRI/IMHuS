# IMHuS

# Presentation 



# Requirements 
ROS Melodic
Gazebo 9
Tiago simulation packages for gazebo : https://github.com/pal-robotics
ORCA : https://github.com/sybrenstuvel/Python-RVO2 (to cite)

# Example 

launch Gazebo :
> roslaunch imhus_gazebo gazebo.launch

add “gui:=true” for graphics

add “robot:=true” for tiago

add “nb_humans:=??” with ?? the number of humans (default=5)


launch IMHuS :
> roslaunch imhus imhus.launch

In the small terminal (boss) that opens,  select Scenarios (1) then the first scenario (1).


***
# TIPS 
## For testing a robot 
communication with IMHuS through AsyncAction like in example video, 
a script that publishes the pose and twist of the robot to the ORCA node

## To add more humans 
comment or uncomment them in the file simulator/gazebo/world/elevator.world + set the parameter nb_humans when launching Gazebo


## To reset humans pose in gazebo
publish an empty msg to /gazebo/reset
## To set the pose of the robot
> rosservice call /gazebo/set_model_state 
with “tiago” as model_name and fill the other fields. 

## To change the simuator
add a folder in src/simulators/ with the launch files, the models of the world and of the agents
the relevant topics that bridge IMHuS to the simulator are called /imhus/input/* or /imhus/output/* (one for each agent). IMHuS takes in input the odometry of each agent (Pose+Twist) and outputs a command (Twist). For now Orca takes in input this twist and send an altered command, either /orca/output/*/cmd_vel or /orca/output/*/cmd_pose (Twist or Pose)

## Update the following
Add radius to TrackedAgent.msg

```
# states
int8 STATIC=0
int8 MOVING=1
int8 STOPPED=2
int8 BLOCKED=3

uint64              track_id
int8                state
int8                type
string              name
float32             radius
TrackedSegment[]    segments

```

Remove the gazebo launch in the tiago_gazebo.launch in Tiago simulator for gazebo. Comment the following part
```
  <include file="$(find pal_gazebo_worlds)/launch/pal_gazebo.launch">
    <arg name="world" value="$(arg world)"/>
    <arg name="gui" value="$(arg gui)"/>
    <arg name="debug" value="$(arg debug)"/>
    <arg name="recording" value="$(arg recording)"/>
    <arg name="extra_gz_model_path" value=""/>
    <arg name="extra_gz_resource_path" value=""/>
    <arg name="extra_gazebo_args" value="$(arg extra_gazebo_args)"/>
  </include>
  
```
