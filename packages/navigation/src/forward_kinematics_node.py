#!/usr/bin/env python3
import os
import rospy
import math
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped
from geometry_msgs.msg import TwistWithCovarianceStamped

class ForwardKinematics(DTROS):
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(ForwardKinematics, self).__init__(node_name=node_name, node_type=NodeType.LOCALIZATION)

        self.r = 0.033
        self.l = 0.0484
        self.encoder_resolution = 135
        self.publish_period = 0.1111
        self.right_tick = 0
        self.left_tick = 0
        self.last_right_tick = 0
        self.last_left_tick = 0

        # subscriber to right encoder topic
        self.sub_right_encoder = rospy.Subscriber('/duckiebot4/right_wheel_encoder_node/tick', WheelEncoderStamped, self.cb_right_wheel)

        # subscriber to left encoder topic
        self.sub_left_encoder = rospy.Subscriber('/duckiebot4/left_wheel_encoder_node/tick', WheelEncoderStamped, self.cb_left_wheel)

        rospy.Timer(rospy.Duration(self.publish_period), self.cb_timer)

        # publishers
        self.pub_rigth_phi = rospy.Publisher('~right_phi/twist', TwistWithCovarianceStamped, queue_size=2)
        self.pub_left_phi = rospy.Publisher('~left_phi/twist', TwistWithCovarianceStamped, queue_size=2)
        self.pub_vel = rospy.Publisher('~duckiebot_base/twist', TwistWithCovarianceStamped, queue_size=2)

    def cb_right_wheel(self, right_tick_msg):
        
        self.right_tick = right_tick_msg.data
    
    def cb_left_wheel(self, left_tick_msg):
        
        self.left_tick = left_tick_msg.data

    def cb_timer(self, event):
        
        phi_dot_right = (self.right_tick - self.last_right_tick) * 2 * math.pi / (self.encoder_resolution * self.publish_period)
        phi_dot_left = (self.left_tick - self.last_left_tick) * 2 * math.pi / (self.encoder_resolution * self.publish_period)

        if self.pub_rigth_phi.anybody_listening():

            right_twist = TwistWithCovarianceStamped()
            right_twist.header.stamp = rospy.get_rostime()
            right_twist.header.frame_id = "duckiebot4/right_wheel_axis"
            right_twist.twist.twist.linear.x = phi_dot_right * self.r
            right_twist.twist.twist.linear.y = 0
            right_twist.twist.covariance = [2e-08, 0, 0, 0, 0, 0,     # x
                                            0, 2e-08, 0, 0, 0, 0,     # y
                                            0, 0, 2e-08, 0, 0, 0,     # z
                                            0, 0, 0, 2e-08, 0, 0,     # roll
                                            0, 0, 0, 0, 2e-08, 0,     # pitch
                                            0, 0, 0, 0, 0, 2e-08]    # yaw
            self.pub_rigth_phi.publish(right_twist)

        if self.pub_left_phi.anybody_listening():

            left_twist = TwistWithCovarianceStamped()
            left_twist.header.stamp = rospy.get_rostime()
            left_twist.header.frame_id = "duckiebot4/left_wheel_axis"
            left_twist.twist.twist.linear.x = phi_dot_left * self.r
            right_twist.twist.twist.linear.y = 0
            left_twist.twist.covariance = [2e-08, 0, 0, 0, 0, 0,     # x
                                            0, 2e-08, 0, 0, 0, 0,     # y
                                            0, 0, 2e-08, 0, 0, 0,     # z
                                            0, 0, 0, 2e-08, 0, 0,     # roll
                                            0, 0, 0, 0, 2e-08, 0,     # pitch
                                            0, 0, 0, 0, 0, 2e-08]    # yaw
            self.pub_left_phi.publish(left_twist)

        twist_msg = TwistWithCovarianceStamped()
        twist_msg.header.stamp = rospy.get_rostime()
        twist_msg.header.frame_id = "duckiebot4/footprint"
        twist_msg.twist.twist.linear.x = (self.r / 2) * (phi_dot_right + phi_dot_left)
        twist_msg.twist.twist.linear.y = 0
        twist_msg.twist.twist.angular.z = self.r * (phi_dot_right - phi_dot_left) / (2 * self.l)
        twist_msg.twist.covariance = [2e-08, 0, 0, 0, 0, 0,     # x
                                      0, 2e-08, 0, 0, 0, 0,     # y
                                      0, 0, 2e-08, 0, 0, 0,     # z
                                      0, 0, 0, 2e-08, 0, 0,     # roll
                                      0, 0, 0, 0, 2e-08, 0,     # pitch
                                      0, 0, 0, 0, 0, 2e-08]    # yaw

        self.pub_vel.publish(twist_msg)

        self.last_right_tick = self.right_tick
        self.last_left_tick = self.left_tick

if __name__ == '__main__':
    node = ForwardKinematics(node_name='forward_kinematics_node')
    rospy.spin()