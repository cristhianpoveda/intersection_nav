#!/usr/bin/env python3
import os
import rospy
import math
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped, Twist2DStamped, WheelsCmdStamped

class ForwardKinematics(DTROS):
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(ForwardKinematics, self).__init__(node_name=node_name, node_type=NodeType.LOCALIZATION)

        self.r = rospy.get_param('~r')
        self.l = rospy.get_param('~l')
        self.encoder_resolution = rospy.get_param('~encoder_resolution')
        self.publish_period = rospy.get_param('~publish_period')
        self.filter_a1 = rospy.get_param('~filter_a1')
        self.filter_b1 = rospy.get_param('~filter_b1')
        self.filter_b2 = rospy.get_param('~filter_b2')
        self.right_tick = 0
        self.left_tick = 0
        self.last_right_tick = 0
        self.last_left_tick = 0
        self.prev_filtered_r = 0
        self.prev_r = 0
        self.prev_filtered_l = 0
        self.prev_l = 0

        # subscriber to right encoder topic
        self.sub_right_encoder = rospy.Subscriber('/duckiebot4/right_wheel_encoder_node/tick', WheelEncoderStamped, self.cb_right_wheel)

        # subscriber to left encoder topic
        self.sub_left_encoder = rospy.Subscriber('/duckiebot4/left_wheel_encoder_node/tick', WheelEncoderStamped, self.cb_left_wheel)

        # publishers
        self.pub_vel = rospy.Publisher('~velocity', Twist2DStamped, queue_size=2)

        self.pub_wheels_speed = rospy.Publisher('~measured/wheels', WheelsCmdStamped, queue_size=2)

        rospy.Timer(rospy.Duration(self.publish_period), self.cb_timer)

    def cb_right_wheel(self, right_tick_msg):
        
        self.right_tick = right_tick_msg.data
    
    def cb_left_wheel(self, left_tick_msg):
        
        self.left_tick = left_tick_msg.data

    # def filter_signal(self, prev_y, x, prev_x):
        
    #     # Filter signal: low-pass filter 50Hz
    #     filtered = round(self.filter_a1 * prev_y + self.filter_b1 * x + self.filter_b2 * prev_x, 4)
    #     return filtered

    def cb_timer(self, event):
        
        phi_dot_right = (self.right_tick - self.last_right_tick) * 2 * math.pi / (self.encoder_resolution * self.publish_period)
        phi_dot_left = (self.left_tick - self.last_left_tick) * 2 * math.pi / (self.encoder_resolution * self.publish_period)

        # filtered_right = self.filter_signal(self.prev_filtered_r, phi_dot_right, self.prev_r)
        # filtered_left = self.filter_signal(self.prev_filtered_l, phi_dot_left, self.prev_l)

        wheels_msg = WheelsCmdStamped()
        wheels_msg.vel_left = phi_dot_left
        wheels_msg.vel_right = phi_dot_right

        self.pub_wheels_speed.publish(wheels_msg)

        twist_msg = Twist2DStamped()
        twist_msg.header.stamp = rospy.get_rostime()
        twist_msg.v = (self.r / 2) * (phi_dot_right + phi_dot_left)
        twist_msg.omega = self.r * (phi_dot_right - phi_dot_left) / (2 * self.l)

        self.pub_vel.publish(twist_msg)

        self.last_right_tick = self.right_tick
        self.last_left_tick = self.left_tick
        self.prev_filtered_r = phi_dot_right
        self.prev_r = phi_dot_right
        self.prev_filtered_l = phi_dot_left
        self.prev_l = phi_dot_left

if __name__ == '__main__':
    node = ForwardKinematics(node_name='forward_kinematics_node')
    rospy.spin()