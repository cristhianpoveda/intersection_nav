#!/usr/bin/env python3
import os
import math
import rospy
import tf
from duckietown.dtros import DTROS, NodeType
from nav_msgs.msg import Odometry

class OdomToMap(DTROS):
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(OdomToMap, self).__init__(node_name=node_name, node_type=NodeType.LOCALIZATION)

        self.publish_period = rospy.get_param('~publish_period')

        rospy.Timer(rospy.Duration(self.publish_period), self.cb_timer)

    def cb_timer(self, duckie_pose):

        br = tf.TransformBroadcaster()
        br.sendTransform((0.0, 0.0, 0),
                        tf.transformations.quaternion_from_euler(0, 0, 0),
                        rospy.Time.now(),
                        "odom",
                        "map")

if __name__ == '__main__':
    node = OdomToMap(node_name='odom_to_map_node')
    rospy.spin()