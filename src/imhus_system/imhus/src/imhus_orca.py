#!/usr/bin/env python

import os
import rospy
import numpy as np
from geometry_msgs.msg import Pose, Twist, PoseStamped
from nav_msgs.msg import Odometry
import message_filters
import rvo2
import math
from transformations import euler_from_quaternion

PI = math.pi
RATE_HZ = 20.0 #20.0

# SIM = rvo2.PyRVOSimulator(1/60.0, 1.5, 3.0 , 1.5, 1.0, 0.70, 1.8) #TODO:better way to give these parameters

#orca param #timestep, neighborDist, #maxNeighbors, timeHorizon, #timeHorizonObst, float radius, float maxSpeed
#orcaparam = value #default

timestep = 1./40.
neighborDist = 2.
maxNeighbors = 5
agent_radius = 0.3
timeHorizon = 2.
timeHorizonObst = 1.
agent_max_speed = 3.0

max_d_angle = 0.1

forcing_orca_th = 20 #threshold to override ORCA, it usually happens for top vertical motions

class Human(object):
    def __init__(self, id, rate=RATE_HZ):
        self.id = id
        self.handler = None
        self.rate = rate
        self.is_moving = False
        self.pose = Pose()
        self.prefVel = Twist()

        #SUB
        self.pose_sub_ = rospy.Subscriber("/imhus/input/"+id+"/odom", Odometry, self.poseCB, queue_size=10)
        self.prefvel_sub_ = rospy.Subscriber("/imhus/output/"+id+"/pref_vel", Twist, self.prefVelCB, queue_size=10)
        #PUB
        self.cmd_pose_pub_ = rospy.Publisher("/orca/output/"+id+"/cmd_pose", Pose, queue_size=10)
        self.twist_cmd_pose_pub_ = rospy.Publisher("/orca/output/"+id+"/cmd_twist", Twist, queue_size=10)

        self.last_prefVel = rospy.Time.now()


    def poseCB(self, msg):
        self.pose = msg.pose.pose

    def prefVelCB(self, msg):
        self.prefVel = msg
        self.last_prefVel = rospy.Time.now()

    def update(self):
        if(rospy.Time.now()>self.last_prefVel+rospy.Duration(1/self.rate)):
            # reinit twist if agents is not moving
            self.prefVel = Twist()
            self.is_moving = False
        else:
            self.is_moving = True

    def publish_cmd(self, orca_pose, orca_twist):
        cmd_pose_ = Pose()
        cmd_twist_ = Twist()
        q = (np.linalg.norm([self.prefVel.linear.x, self.prefVel.linear.y])- np.linalg.norm([orca_twist[0], orca_twist[1]]))**2
        if q > forcing_orca_th:
            # print(self.id, q)
            #this case means ORCA is making the agent static probably pointlessly, so we override it
            rospy.logwarn("%s is not using ORCA and relies only on the global plan! It may collide. \n This is temporary.", self.id)
            cmd_pose_.position.x = self.pose.position.x + (1./RATE_HZ)*self.prefVel.linear.x*0.1
            cmd_pose_.position.y = self.pose.position.y + (1./RATE_HZ)*self.prefVel.linear.y*0.1
            angle = math.atan2(self.prefVel.linear.x, self.prefVel.linear.y)
            cmd_pose_.orientation.x, cmd_pose_.orientation.y, cmd_pose_.orientation.z, cmd_pose_.orientation.w = euler_to_quaternion(angle, 0, 0)
            cmd_twist_.linear.x = self.prefVel.linear.x*0.1
            cmd_twist_.linear.y = self.prefVel.linear.y*0.1
        else:
            if(np.linalg.norm([orca_twist[0], orca_twist[1]])>0.01):
                cmd_pose_.position.x = orca_pose[0]
                cmd_pose_.position.y = orca_pose[1]
                #for the angle either we deduce it from the twit, or we apply the one from the global plan stored in the twist send by imhus
                next_angle = normalize(math.atan2(orca_twist[1], orca_twist[0]))
                prev_angles = euler_from_quaternion([self.pose.orientation.w, self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z])
                prev_angle = normalize(prev_angles[2])
                v1 = [np.cos(next_angle), np.sin(next_angle)]
                v2 = [np.cos(prev_angle), np.sin(prev_angle)]
                dot_product = np.dot(v1, v2)
                d_angle = np.arccos(dot_product)

                if next_angle < PI:
                    if prev_angle > next_angle and prev_angle < normalize(next_angle+PI):
                        d_angle = -d_angle
                elif next_angle > PI:
                    if prev_angle > next_angle or prev_angle < normalize(next_angle+PI):
                        d_angle = -d_angle
                else:
                    if prev_angle > PI:
                        d_angle = -d_angle

                d_angle = max(d_angle,-max_d_angle)
                d_angle = min(d_angle, max_d_angle)
                next_angle = prev_angle + d_angle
                cmd_pose_.orientation.x, cmd_pose_.orientation.y, cmd_pose_.orientation.z, cmd_pose_.orientation.w = euler_to_quaternion(next_angle, 0, 0)

                #for cohan only
                cmd_twist_.linear.x = orca_twist[0]
                cmd_twist_.linear.y = orca_twist[1]
                cmd_twist_.angular.z = 0 #TODO
            else:
                cmd_pose_ = self.pose

        self.cmd_pose_pub_.publish(cmd_pose_)
        self.twist_cmd_pose_pub_.publish(cmd_twist_)



class Robot(object):
    def __init__(self, id, rate=RATE_HZ):
        self.id = id
        self.handler = None
        self.rate = rate
        self.is_moving = False
        self.pose = Pose()
        self.prefVel = Twist()

        #SUB
        self.odom_sub_ = rospy.Subscriber("/odom", Odometry, self.odomCB, queue_size=1)

    def odomCB(self, msg):
        self.pose = msg.pose.pose
        self.prefVel = msg.twist.twist

    #method only for humans
    def update(self):
        pass

    def publish_cmd(self, orca_pose, orca_twist):
        pass

###########################################################################################
def normalize(theta):
    result = math.fmod(theta , 2.0*PI)
    if result < 0:
        # print(result + 2.0*PI)
        return result + 2.0*PI
    # print(result)
    return result

def euler_to_quaternion(yaw, pitch, roll):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]

def add_obstacles_to_sim(sim):
    #polygons : 2D points, anticlowise
    #MAP := ELEVATOR
    sim.addObstacle([(-10, 10), (10, 10)]) #map walls
    sim.addObstacle([(10, 10), (10, -10)])
    sim.addObstacle([(10, -10), (-10, -10)])
    sim.addObstacle([(-10, -10), (-10, 10)])
    sim.addObstacle([(0, 10), (0, 1.2)]) #room separation walls
    sim.addObstacle([(0, -10), (0, -1.2)])
    sim.addObstacle([(-1.2, 1), (1.2, 1)]) #elevator walls
    sim.addObstacle([(-1.2, -1), (1.2, -1)])
    # sim.addObstacle([(1.5, -0.117), (5.5, -0.177), (5.5, -0.0143), (1.5, -0.0143)]) #
    ###########################################""
    #TODO add obstacles like tables and chairs
    sim.addObstacle([(3.5, 4), (3.5, 1.5), (7.2, 1.5), (7.2, 4)])
    sim.addObstacle([(-7.2, 4), (-7.2, 1.5), (-3.5, 1.5), (-3.5, 4)])
    sim.addObstacle([(2.3, -4), (2.3, -6.5), (7, -6.5), (7, -4)])
    sim.addObstacle([(7, -4), (7, -6.5), (2.3, -6.5), (2.3, -4)])


    #!!!! !!!! !!!!####################
    sim.processObstacles() #this line is imoportant !
    print("Obstacles added to ORCA sim.")


###################################################
###################################################
def main():
    node_id = "ORCA_IMHUS_node"
    rospy.init_node(node_id)
    r = rospy.Rate(RATE_HZ)
    n = rospy.get_param("nb_humans", 6)
    robot = rospy.get_param("robot", False)

    rospy.logwarn("ORCA_IMHUS_node starts...")

    agents = []

    orca_sim = rvo2.PyRVOSimulator(timestep, neighborDist, maxNeighbors, timeHorizon, timeHorizonObst, agent_radius, agent_max_speed)
    #orca param #timestep, neighborDist, #maxNeighbors, timeHorizon, #timeHorizonObst, float radius, float maxSpeed

    add_obstacles_to_sim(orca_sim)

    for i in range(1,n+1):
        name = "human"+str(i)
        agents.append(Human(name))
        agents[-1].handler = orca_sim.addAgent((0,0))

    if(robot): #only if there is a robot
        agents.append(Robot("robot"))
        agents[-1].handler = orca_sim.addAgent((0,0))

    for agent in agents:
        rospy.loginfo("%s tracks agent : %s", node_id, agent.id)



    while not rospy.is_shutdown():

        for agent in agents:
            agent.update()
            orca_sim.setAgentPosition(agent.handler, (agent.pose.position.x,agent.pose.position.y))
            orca_sim.setAgentPrefVelocity(agent.handler, (agent.prefVel.linear.x,agent.prefVel.linear.y))

        orca_sim.doStep()

        for agent in agents:
            if True: #agent.is_moving:
                agent.publish_cmd(orca_sim.getAgentPosition(agent.handler), orca_sim.getAgentVelocity(agent.handler))

        r.sleep()



if __name__ == "__main__":
    main()
