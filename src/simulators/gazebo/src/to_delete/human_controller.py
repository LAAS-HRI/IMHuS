#! /usr/bin/env python
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, String
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, SetModelState, SetLinkState
from gazebo_msgs.msg import ModelState, LinkState
import rospy
from geometry_msgs.msg import Twist, Pose
import numpy as np
import math
from scipy.spatial.transform import Rotation
from move_base_msgs.msg import MoveBaseActionGoal

############################################################################################################
###this class control the human in gazebo
###applyCmd() uses a topic to give a twist to the agent, applyCmd2() uses a service
###if several humans are to be moved at the same time we may need the topic because services work serially.#
############################################################################################################
class humanController(object):
    def __init__(self, name):
        rospy.wait_for_service('/gazebo/set_model_state')
        rospy.wait_for_service('/gazebo/get_model_state')
        #self.name = name
        self.controlled_human = name
        self.setState = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)
        self.getState = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        self.cmd_vel_sub = rospy.Subscriber('/gazebo_agents/human1/cmd_vel', Twist, self.cmd_velCB)
        self.twist_pub = rospy.Publisher('/gazebo/set_link_state', LinkState, queue_size=10)

        rospy.Subscriber("/controlled_entity", String, self.shiftControlledHumanCB)
        # self.goal_pub = rospy.Publisher('/human_controller/goal_state', String, queue_size=10)


        self.state = 0
        
        self.goal_topic = rospy.Subscriber('/human1/move_base/goal', MoveBaseActionGoal, self.newGoal)
        self.goal = 0

        self.updateState()

    def newGoal(self, msg):
        self.goal = msg.goal.target_pose.pose

    def shiftControlledHumanCB(self, msg):
        #first we stop moving curretn agent, it is important
        self.state.twist.linear.x = self.state.twist.linear.y = self.state.twist.linear.z = 0
        self.state.twist.angular.x = self.state.twist.angular.y = self.state.twist.angular.z = 0
        self.applyCmd()

        #then we change agent
        self.controlled_human = msg.data
        self.updateState()
        
        #rospy.sleep(10)
        rospy.logwarn("Now controlling %s" %self.controlled_human)


    def cmd_velCB(self, msg):
        self.updateState()
        # self.last_time = self.current_time
        # self.current_time = rospy.Time.now()
        #self.dt = self.current_time - self.last_time
        self.state.twist = msg
        self.applyCmd2()

    def updateState(self):
        model = GetModelStateRequest()
        model.model_name = self.controlled_human
        safe_copy = self.state
        try:
            self.state = self.getState(model)
        except:
            self.state = safe_copy

    def applyCmd(self):
        final_cmd = LinkState()
        final_cmd.link_name = self.controlled_human+'::link'

        final_cmd.pose = self.state.pose

        v = np.sqrt(self.state.twist.linear.x ** 2 + self.state.twist.linear.y ** 2)
        # angles has to be xyzw
        angles = [self.state.pose.orientation.x, self.state.pose.orientation.y, self.state.pose.orientation.z,
                  self.state.pose.orientation.w]
        rot = Rotation.from_quat(angles)
        rot_euler = rot.as_euler('xyz', degrees=False)
        theta = rot_euler[2]

        final_cmd.twist = self.state.twist
        final_cmd.twist.linear.x = v * np.cos(theta)
        final_cmd.twist.linear.y = v * np.sin(theta)

        # make sure the movement is planar
        final_cmd.twist.angular.x = 0
        final_cmd.twist.angular.y = 0
        final_cmd.twist.linear.z = 0



        d = np.sqrt((self.state.pose.position.x-self.goal.position.x)**2+(self.state.pose.position.y-self.goal.position.y)**2)
        if d<0.3:
            #this part is a cheat code to avoid weird behaviours when close to the goal
            final_cmd.pose.position = self.goal.position
            final_cmd.pose.orientation = self.goal.orientation
            final_cmd.reference_frame = ''
            final_cmd.twist.linear.x = 0
            final_cmd.twist.linear.y = 0
            final_cmd.twist.linear.z = 0
            final_cmd.twist.angular.x = 0
            final_cmd.twist.angular.y = 0
            final_cmd.twist.angular.z = 0
            msg = String()
            msg.data = "goal_done"
            # self.goal_pub.publish(msg)

        #modify agent state
        self.twist_pub.publish(final_cmd)



        self.updateState()

    def applyCmd2(self):
        final_cmd = LinkState()
        final_cmd.link_name = self.controlled_human+'::link'

        #final_cmd.pose = self.state.pose

        v = np.sqrt(self.state.twist.linear.x ** 2 + self.state.twist.linear.y ** 2)
        # angles has to be xyzw
        angles = [self.state.pose.orientation.x, self.state.pose.orientation.y, self.state.pose.orientation.z,
                  self.state.pose.orientation.w]
        rot = Rotation.from_quat(angles)
        rot_euler = rot.as_euler('xyz', degrees=False)
        theta = rot_euler[2]

        final_cmd.twist = self.state.twist
        # final_cmd.twist.linear.x = self.state.twist.linear.x * np.cos(theta) - self.state.twist.linear.y * np.sin(theta)
        # final_cmd.twist.linear.y = self.state.twist.linear.x * np.sin(theta) - self.state.twist.linear.y * np.cos(theta)

        # make sure the movement is planar
        final_cmd.twist.angular.x = 0
        final_cmd.twist.angular.y = 0
        final_cmd.twist.linear.z = 0

        final_cmd.reference_frame = final_cmd.link_name

        d = np.sqrt((self.state.pose.position.x-self.goal.position.x)**2+(self.state.pose.position.y-self.goal.position.y)**2)
        if d<0.3:
            #this part is a cheat code to avoid weird behaviours when close to the goal
            final_cmd.pose.position = self.goal.position
            final_cmd.pose.orientation = self.goal.orientation
            final_cmd.reference_frame = ''
            final_cmd.twist.linear.x = 0
            final_cmd.twist.linear.y = 0
            final_cmd.twist.linear.z = 0
            final_cmd.twist.angular.x = 0
            final_cmd.twist.angular.y = 0
            final_cmd.twist.angular.z = 0

        #modify agent state
        self.setState(final_cmd)



        self.updateState()


if __name__ == '__main__':
    rospy.init_node('human_controller')

    human_controller = humanController('human1')

    r = rospy.Rate(20)
    while not rospy.is_shutdown():
        r.sleep()
