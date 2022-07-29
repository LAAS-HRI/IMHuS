#!/usr/bin/env python

# This node publishes the odom of the robot (important for IMHuS), the TF (important for the tested robot software like MoveBase)
#  and also the marker to rviz (this could be moved in the robot.cpp of IMHuS, as it is for the humans right now)
from nav_msgs.msg import Odometry
import rospy
import tf
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from gazebo_msgs.msg import ModelState
from visualization_msgs.msg import MarkerArray, Marker


class BroadCastTF(object):
    def __init__(self, name):

        self.ns = name
        self.br = tf.TransformBroadcaster()
        rospy.wait_for_service('/gazebo/get_model_state')
        self.getState = rospy.ServiceProxy(
            '/gazebo/get_model_state', GetModelState)
        self.State = ModelState()
        self.initState = ModelState()

        self.odom_pub_ = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.marker_pub_ = rospy.Publisher("/marker_robot_pose", MarkerArray ,queue_size=10)
        self.markers_ = MarkerArray()

        # RVIZ
        marker = Marker()
        marker.header.frame_id = "map"
        marker.pose.orientation.w = 1
        marker.pose.orientation.z = 1
        marker.pose.orientation.y = 0
        marker.pose.orientation.x = 0
        # arrow
        marker.type = 0
        marker.id = 0
        marker.pose.position.z = 0.1
        marker.scale.x = 1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 1
        marker.color.g = 0
        marker.color.b = 0
        marker.color.a = 1
        self.markers_.markers.append(marker)
        # CYLINDER
        marker = Marker()
        marker.header.frame_id = "map"
        marker.pose.orientation.w = 1
        marker.pose.orientation.z = 1
        marker.pose.orientation.y = 0
        marker.pose.orientation.x = 0
        marker.type = 3
        marker.id = 1
        marker.pose.position.z = 0.9
        marker.scale.x = 0.6
        marker.scale.y = 0.6
        marker.scale.z = 1.8
        marker.color.r = 1
        marker.color.g = 0
        marker.color.b = 0
        marker.color.a = 1
        self.markers_.markers.append(marker)
        self.init = False
        rospy.sleep(0.5)

    def updateState(self):
        model = GetModelStateRequest()
        model.model_name = self.ns
        try:
            self.State = self.getState(model)

            self.BroadcastTF()

        except(Exception):
            rospy.logerr(str(Exception))

    def BroadcastTF(self):
        now = rospy.Time.now()

        if(self.State.success):
            # for fake loca
            self.br.sendTransform((self.State.pose.position.x, self.State.pose.position.y, 0),
                                  (self.State.pose.orientation.x, self.State.pose.orientation.y,
                                   self.State.pose.orientation.z, self.State.pose.orientation.w),
                                  now,
                                  '/base_footprint',
                                  '/odom')

            self.br.sendTransform((0, 0, 0),
                                    (0, 0,
                                    0, 1),
                                    now,
                                    '/odom',
                                    '/map')
            self.UpdateMarker()
            self.PubOdom()

    def UpdateMarker(self):
        self.markers_.markers[0].pose.position.x = self.State.pose.position.x
        self.markers_.markers[0].pose.position.y = self.State.pose.position.y
        self.markers_.markers[1].pose.position.x = self.State.pose.position.x
        self.markers_.markers[1].pose.position.y = self.State.pose.position.y
        #self.markers_.markers[2].pose.position.x = self.State.pose.position.x
        #self.markers_.markers[2].pose.position.y = self.State.pose.position.y
        self.markers_.markers[0].pose.orientation.w = self.State.pose.orientation.w
        self.markers_.markers[0].pose.orientation.x = self.State.pose.orientation.x
        self.markers_.markers[0].pose.orientation.y = self.State.pose.orientation.y
        self.markers_.markers[0].pose.orientation.z = self.State.pose.orientation.z

        self.marker_pub_.publish(self.markers_)

    def PubOdom(self):
        msg = Odometry()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = self.State.pose.position.x
        msg.pose.pose.position.y = self.State.pose.position.y
        msg.pose.pose.position.z = 0
        msg.pose.pose.orientation.x = self.State.pose.orientation.x
        msg.pose.pose.orientation.y = self.State.pose.orientation.y
        msg.pose.pose.orientation.z = self.State.pose.orientation.z
        msg.pose.pose.orientation.w = self.State.pose.orientation.w

        msg.twist.twist.linear.x = self.State.twist.linear.x
        msg.twist.twist.linear.y = self.State.twist.linear.y
        msg.twist.twist.angular.z = self.State.twist.angular.z

        self.odom_pub_.publish(msg)


if __name__ == '__main__':
    rospy.init_node('tf_BC')

    robot_agent_tf = BroadCastTF('tiago')

    r = rospy.Rate(20)
    while not rospy.is_shutdown():
        robot_agent_tf.updateState()
        r.sleep()
