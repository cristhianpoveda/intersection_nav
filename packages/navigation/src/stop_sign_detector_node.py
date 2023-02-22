#!/usr/bin/env python3

import numpy as np
import rospy
import time
import math
import cv2
from cv_bridge import CvBridge, CvBridgeError
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, CameraInfo
from duckietown_msgs.msg import FSMState
from std_msgs.msg import Float32
from intersection_msgs.srv import DetectStopSign, DetectStopSignResponse

class StopSignDetector(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(StopSignDetector, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        # params
        self.red_lower_1 = rospy.get_param('~red_lower_1')
        self.red_upper_1 = rospy.get_param('~red_upper_1')
        self.red_lower_2 = rospy.get_param('~red_lower_2')
        self.red_upper_2 = rospy.get_param('~red_upper_2')
        self._detection_distance = rospy.get_param('~detection_distance')
        self._stop_distance = rospy.get_param('~stop_distance')
        self.lane_width = rospy.get_param('~lane_width')
        self.fx = rospy.get_param('~fx')
        self.fy = rospy.get_param('~fy')
        self.cx = rospy.get_param('~cx')
        self.cy = rospy.get_param('~cy')
        self.k1 = rospy.get_param('~k1')
        self.k2 = rospy.get_param('~k2')
        self.k3 = rospy.get_param('~k3')
        self.k4 = rospy.get_param('~k4')
        self.img_w = rospy.get_param('~img_w')
        self.img_h = rospy.get_param('~img_h')
        self.camera_h = rospy.get_param('~camera_h')

        self.K = np.array([[self.fx,0.0,self.cx],[0.0,self.fy,self.cy],[0.0,0.0,1.0]])
        self.D=np.array([[self.k1],[self.k2],[self.k3],[self.k4]])

        self.newmatrix, roi = cv2.getOptimalNewCameraMatrix(self.K, self.D, (self.img_w,self.img_h), 1, (self.img_w,self.img_h))

        self.x_r, self.y_r, self.w_r, self.h_r = roi

        self.focal = np.sqrt(np.power(self.fx, 2) + np.power(self.fy, 2)) / 2
        
        self.image_np = np.zeros((self.img_w, self.img_h, 1), dtype = "uint8")

        self.stop = False

        # detect stop sign service
        self.srv_stop_sign = rospy.Service('~detect_stop_sign', DetectStopSign, self.srv_stop)

        # publisher (edited image)
        self.pub_center = rospy.Publisher('~debug/detected_center/compressed', CompressedImage, queue_size=1)

        self.pub_dist = rospy.Publisher('~debug/stop_line_distance', Float32, queue_size=1)

        self.pub_time = rospy.Publisher('~debug/calculation_time', Float32, queue_size=1)

        # publisher (FSM State)
        self.pub_state = rospy.Publisher('/duckiebot4/fsm_node/mode', FSMState, queue_size=1)

        self.bridge = CvBridge()

        # subscriber to camera_node/image/compressed
        self.sub = rospy.Subscriber('/duckiebot4/camera_node/image/compressed', CompressedImage, self.camera, queue_size=1)

    def undistort(self, img):
        
        undistorted_img = cv2.undistort(img, self.K, self.D, None, self.newmatrix)
        undistorted_img = undistorted_img[self.y_r:self.y_r+self.h_r, self.x_r:self.x_r+self.w_r]
        resized_img = cv2.resize(undistorted_img, (self.img_w, self.img_h), cv2.INTER_LINEAR)
        return resized_img

    def srv_stop(self, req=None):
        
        distance_base = self._detection_distance
        x,y,w,h = 1,1,2,2
        
        while not self.stop:

            start = rospy.get_rostime()

            undistorted_img = self.undistort(self.image_np)

            # hsv image
            imghsv = cv2.cvtColor(undistorted_img, cv2.COLOR_BGR2HSV)

            # HSV red color range
            red_lower_1 = np.array(self.red_lower_1)
            red_upper_1 = np.array(self.red_upper_1)
            red_lower_2 = np.array(self.red_lower_2)
            red_upper_2 = np.array(self.red_upper_2)

            mask1 = cv2.inRange(imghsv, red_lower_1, red_upper_1)
            mask2 = cv2.inRange(imghsv, red_lower_2, red_upper_2)
 
            red_mask = mask1 + mask2

            contours, hierachy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            if len(contours) != 0:
                c = max(contours, key = cv2.contourArea)

                x,y,w,h = cv2.boundingRect(c)

                distance_camera = self.focal * self.lane_width / w

                distance_base = np.sqrt(np.power(distance_camera, 2) - np.power(self.camera_h, 2)) + 0.058

                end = rospy.get_rostime()
                time_t = (end - start).to_sec()

                if self.pub_time.anybody_listening():

                    self.pub_time.publish(time_t)

                if distance_base < self._stop_distance:
                    # pub FSM state to stop
                    state = FSMState()
                    state.state = "INTERSECTION_COORDINATION"
                    self.pub_state.publish(state)
                    time.sleep(1)
                    self.stop = True


                if self.pub_dist.anybody_listening():

                    str_msg = distance_base
                    self.pub_dist.publish(str_msg)

            else:
                distance_base = 0.5

            if self.pub_center.anybody_listening():

                image_center = cv2.rectangle(undistorted_img,(x,y),(x+w,y+h),(0,255,0),2)

                try:
                    self.pub_center.publish(self.bridge.cv2_to_compressed_imgmsg(image_center, "jpeg"))
                except CvBridgeError as e:
                    print(e)

        rospy.loginfo("Duckiebot stopped at: %f m from stop line.", distance_base)

        response = DetectStopSignResponse()
        response.distance.data = distance_base

        self.stop = False

        return response
    
    def camera(self, image):

        try:
            self.image_np = self.bridge.compressed_imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    # create the node
    node = StopSignDetector(node_name='stop_sign_detector_node')

    # keep spinning
    rospy.spin()
