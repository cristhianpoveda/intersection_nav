#!/usr/bin/env python3
import os
import rospy
import math
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped, Twist2DStamped
from geometry_msgs.msg import TwistStamped 
from geometry_msgs.msg import PointStamped

class TranslatorNode(DTROS):
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(TranslatorNode, self).__init__(node_name=node_name, node_type=NodeType.LOCALIZATION)

        # subscriber to right encoder topic
        self.sub_right_encoder = rospy.Subscriber('/duckiebot4/right_wheel_encoder_node/tick', WheelEncoderStamped, self.cb_right_wheel)

        # subscriber to left encoder topic
        self.sub_left_encoder = rospy.Subscriber('/duckiebot4/left_wheel_encoder_node/tick', WheelEncoderStamped, self.cb_left_wheel)

        # subscriber to left encoder topic
        self.sub_cmd = rospy.Subscriber('/duckiebot4/car_cmd_switch_node/cmd', Twist2DStamped, self.cb_cmd)

        # publishers
        self.pub_encoder_r = rospy.Publisher('~right/twist', PointStamped, queue_size=2)
        self.pub_encoder_l = rospy.Publisher('~left/twist', PointStamped, queue_size=2)
        self.pub_vel = rospy.Publisher('~duckiebot_base/twist', TwistStamped, queue_size=2)

    def cb_right_wheel(self, right_tick_msg):

        twist_r = PointStamped()
        twist_r.header.stamp = rospy.get_rostime()
        twist_r.point.x = right_tick_msg.data
        self.pub_encoder_r.publish(twist_r)
    
    def cb_left_wheel(self, left_tick_msg):

        twist_l = PointStamped()
        twist_l.header.stamp = rospy.get_rostime()
        twist_l.point.x = left_tick_msg.data
        self.pub_encoder_l.publish(twist_l)

    def cb_cmd(self, cmd_duckie):

        cmd_msg = TwistStamped()
        cmd_msg.header.stamp = rospy.get_rostime()
        cmd_msg.twist.linear.x = cmd_duckie.v
        cmd_msg.twist.angular.z = cmd_duckie.omega
        self.pub_vel.publish(cmd_msg)

if __name__ == '__main__':
    node = TranslatorNode(node_name='translator_node')
    rospy.spin()