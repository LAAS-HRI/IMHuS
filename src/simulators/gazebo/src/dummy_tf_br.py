#!/usr/bin/env python

#This node simply moves a dummy frame called floating_frame just for movebase to update its costmap
#all the rest is obsolete now
#this is okay at long as there exists a human1 in the sim. It may be safer to randomly generate a transform independantly of the humans
import rospy
import tf
from gazebo_msgs.msg import LinkState, LinkStates
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from gazebo_msgs.msg import ModelState

class humans_TF_BR(object):
    def __init__(self):
        self.br = tf.TransformBroadcaster()

        # if rospy.has_param("nb_humans"):
        #     self.nb = int(rospy.get_param("nb_humans"))
        # else:
        #     self.nb = 1
        
        # rospy.loginfo("The TF broadcaster node for humans will broadcast for %s humans", self.nb)
        self.nb = 1

        self.pose_list = []
        self.req = GetModelStateRequest()

        rospy.wait_for_service('/gazebo/get_model_state')
        self.getState = rospy.ServiceProxy(
            '/gazebo/get_model_state', GetModelState)

    def update(self):
        for i in range(1,self.nb+1):
            name = "human" + str(i)
            self.req.model_name = name

            try:
                 self.broadcastTF(self.getState(self.req), name)
            except(Exception):
                rospy.logerr(str(Exception))


    def broadcastTF(self, state, name):
        now = rospy.Time.now()

        if(state.success):
            # self.br.sendTransform((state.pose.position.x, state.pose.position.y, 0),
            #                       (state.pose.orientation.x, state.pose.orientation.y,
            #                        state.pose.orientation.z, state.pose.orientation.w),
            #                       now,
            #                       name+'/base_footprint',
            #                       name+'/odom')

            # self.br.sendTransform((0, 0, 0),
            #                         (0, 0,
            #                         0, 1),
            #                         now,
            #                         name+'/odom',
            #                         "/map")
            
            #it is necessary to have a moving frame to update the costmap
            if(name == "human1"):
                self.br.sendTransform((10, 10, 0),
                                  (state.pose.orientation.x, state.pose.orientation.y,
                                   state.pose.orientation.z, state.pose.orientation.w),
                                  now,
                                  '/floating_frame',
                                  'map')

if __name__ == '__main__':
    rospy.init_node('humans_TF_BR')

    tf_br = humans_TF_BR()

    r = rospy.Rate(20)
    while not rospy.is_shutdown():
        tf_br.update()
        r.sleep()