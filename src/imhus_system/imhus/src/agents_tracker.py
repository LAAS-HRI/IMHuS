#!/usr/bin/python

#this node subscribe to the topics of agents' poses to wrapped every data in a msg
# #this msg is to update the costmap 

import sys
import rospy
import tf2_ros

from cohan_msgs.msg import TrackedAgents, TrackedAgent, AgentMarkerStamped, TrackedSegment, TrackedSegmentType, AgentType
import message_filters
from nav_msgs.msg import Odometry

class AgentsTracker(object):

    def __init__(self):
        rospy.init_node("agentTracker_IMHuS_node", anonymous=True)
        self.nb_hum = rospy.get_param("nb_humans", default=6)
        self.robot_bool = rospy.get_param("robot", default=False)
        self.agent_sub = []

        self.Segment_Type = TrackedSegmentType.TORSO
        self.agents = TrackedAgents()

        # Publisher
        self.tracked_agents_pub = rospy.Publisher("/tracked_agents", TrackedAgents, queue_size=1)

        #subscribe to agents
        for agent_id in range(1,self.nb_hum+1):
            name = 'human'+str(agent_id)
            
            self.agent_sub.append(message_filters.Subscriber("/imhus/input/"+name+"/odom", Odometry))

        #TODO uncomment later (i do this just to use with cohan)
        # if self.robot_bool:
        #     self.agent_sub.append(message_filters.Subscriber("/odom", Odometry))
        self.robot_bool = False

        pose_msg = message_filters.TimeSynchronizer(self.agent_sub, 10)
        pose_msg.registerCallback(self.AgentsCB)
        rospy.spin()


    def AgentsCB(self,*msg):
        tracked_agents = TrackedAgents()
        for agent_id in range(1,self.nb_hum+1):
            agent_segment = TrackedSegment()
            agent_segment.type = self.Segment_Type
            agent_segment.pose.pose = msg[agent_id-1].pose.pose
            agent_segment.twist.twist = msg[agent_id-1].twist.twist
            tracked_agent = TrackedAgent()
            tracked_agent.type = AgentType.HUMAN
            tracked_agent.radius = 0.3
            tracked_agent.name = "human"+str(agent_id)
            tracked_agent.segments.append(agent_segment)
            tracked_agents.agents.append(tracked_agent)

        if self.robot_bool:
            agent_segment = TrackedSegment()
            agent_segment.type = self.Segment_Type
            agent_segment.pose.pose = msg[-1].pose.pose
            agent_segment.twist.twist = msg[-1].twist.twist
            tracked_agent = TrackedAgent()
            tracked_agent.type = AgentType.ROBOT
            tracked_agent.radius = 0.5
            tracked_agent.name = "tiago"
            tracked_agent.segments.append(agent_segment)
            tracked_agents.agents.append(tracked_agent)
        
        # self.agents.agents.append(tracked_agent)
        # # if(tracked_agents.agents):
        # #     self.agents = tracked_agents
        # #     self.sig_1 = True
        tracked_agents.header.stamp = rospy.Time.now()
        tracked_agents.header.frame_id = "map"
        self.tracked_agents_pub.publish(tracked_agents)

    def RobotCB(self, *msg):
        agent_segment = TrackedSegment()
        agent_segment.type = self.Segment_Type
        agent_segment.pose.pose = msg.pose.pose
        agent_segment.twist.twist = msg.twist.twist
        tracked_agent = TrackedAgent()
        tracked_agent.type = AgentType.ROBOT
        tracked_agent.name = "tiago"
        tracked_agent.segments.append(agent_segment)
        self.agents.agents.append(tracked_agent)

    def Pub(self):
        self.tracked_agents_pub.publish(self.agents)


    def update(self):
        # rospy.spin()
        pass









if __name__ == '__main__':
    agentsTracker = AgentsTracker()

    r = rospy.Rate(20)
    while not rospy.is_shutdown():
        agentsTracker.update()
        r.sleep()
