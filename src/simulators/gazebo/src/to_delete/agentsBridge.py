#! /usr/bin/env python
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, SetModelState
from gazebo_msgs.msg import ModelState
import rospy
import tf
from geometry_msgs.msg import Twist, Pose, PoseWithCovarianceStamped
import sys
from cohan_msgs.msg import TrackedAgents, TrackedSegment, TrackedSegmentType,AgentType
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import String

########################################################################
########################################################################
###This class makes two things. It publishes the odom of the agents and broadcast the frames.
###It broadcast the floating frame which is important to control multiple humans with one instance of movebase

class AgentsBridge(object):
    def __init__(self, ns):

        rospy.wait_for_service('/gazebo/get_model_state')
        self.ns = ns
        self.getState = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.State = 0
        self.br = tf.TransformBroadcaster()
        self.odom = Odometry()

        #Subscriber

        #robot publishers
        if self.ns == 'tiago':
            self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
            self.base_pose_pub = rospy.Publisher('/base_pose_ground_truth', Odometry, queue_size=10)
            # self.tracked_agent_pub = rospy.Publisher('/tracked_agents', TrackedAgents, queue_size=10)

        else:
            rospy.logerr("Wrong namespace of one of the agents.")
            sys.exit()

        # need a remapping for robot ? why ?
        if self.ns.startswith('tiago'):
            rospy.Subscriber("/mobile_base_controller/cmd_vel", Twist, self.remap)
            self.remap_pub = rospy.Publisher("/gazebo_agents/"+self.ns+"/cmd_vel", Twist, queue_size=10)


    def updateState(self):
        model = GetModelStateRequest()
        model.model_name = self.ns
        try:
            self.State = self.getState(model)

            self.pubState()
            self.broadcastTF()
            
        except:
            pass

    # def pub_tracked_agent(self):
    #     agent_segment = TrackedSegment()

    #     agent_segment.pose.pose = self.State.pose
    #     agent_segment.twist.twist =  self.State.twist
    #     tracked_agent = TrackedAgent()
        
    #     agent_segment.type = TrackedSegmentType.TORSO
    #     tracked_agent.type = AgentType.HUMAN
    #     tracked_agent.name = "human"

    #     tracked_agent.segments.append(agent_segment)

    #     self.tracked_agent_pub.publish(tracked_agent)

    def pubState(self):        
        self.odom.pose.pose = self.State.pose
        self.odom.twist.twist = self.State.twist
        self.odom.pose.pose.position.z = 0
        self.odom.header.stamp = rospy.Time.now()

        self.odom.header.frame_id = '/'+self.ns+'/odom'
        # self.odom_pub.publish(self.odom)
        # self.base_pose_pub.publish(self.odom)

    def broadcastTF(self):
        if(self.odom):
            now = rospy.Time.now()

            self.br.sendTransform((self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, 0),
                            (self.odom.pose.pose.orientation.x,self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z,self.odom.pose.pose.orientation.w),
                            now,
                            '%s/base_footprint' %self.ns ,
                            '%s/odom' %self.ns )

            self.br.sendTransform((0, 0, 0),
                                    (0, 0,
                                    0, 1),
                                    now,
                                    '%s/odom' %self.ns,
                                    '/map')


    def remap(self, data):
        try:
            self.remap_pub.publish(data)
        except:
            pass

##########################################################
#######################################################
def main():
    rospy.init_node('AgentsBridge', anonymous=True)

    #set tiago to true if it is in simulation
    tiago = True
    list = []

    #add tiago
    if tiago:
        list.append(AgentsBridge("tiago"))


    #run in loop
    r = rospy.Rate(100) #needs a high rate
    while not rospy.is_shutdown():
        for agent in list:
            agent.updateState()

        r.sleep()


if __name__ == '__main__':
    main()
