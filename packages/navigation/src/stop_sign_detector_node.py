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
from std_msgs.msg import String
from std_srvs.srv import EmptyResponse, Empty
from intersection_msgs.srv import DetectStopSign, DetectStopSignResponse

class StopSignDetector(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(StopSignDetector, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        # params
        self._Img_process = rospy.get_param('~Img_process')
        self._detection_distance = rospy.get_param('~detection_distance')
        self._stop_distance = rospy.get_param('~stop_distance')
        self._calculation_rate = rospy.get_param('~calculation_rate')
        
        self.image_np = np.zeros((416, 416, 1), dtype = "uint8")

        self.K = np.array([[322.93673636507424,0.0,327.09788378259907],[0.0,321.852444071687,207.09902413788922],[0.0,0.0,1.0]])
        self.D=np.array([[-0.26239184522108644],[0.04732311174178657],[0.007784609968258371],[-0.0011570519649992076]])

        self.stop = False

        # detect stop sign service
        self.srv_stop_sign = rospy.Service('~detect_stop_sign', DetectStopSign, self.srv_stop)

        # publisher (edited image)
        self.pub_center = rospy.Publisher('~detected_center/compressed', CompressedImage, queue_size=1)

        self.pub_dist = rospy.Publisher('~stop_line_distance', String, queue_size=1)

        # publisher (FSM State)
        self.pub_state = rospy.Publisher('/duckiebot4/fsm_node/mode', FSMState, queue_size=1)

        self.bridge = CvBridge()

        # subscriber to camera_node/image/compressed
        self.sub = rospy.Subscriber('/duckiebot4/camera_node/image/compressed', CompressedImage, self.camera, queue_size=1)

    def undistort(self, img):
    
        h= img.shape[0]
        w = img.shape[1]
        newmatrix, roi = cv2.getOptimalNewCameraMatrix(self.K, self.D,(w,h),1, (w,h))
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(self.K, self.D, np.eye(3),self.K, (w,h), cv2.CV_32FC1)
        undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR)

        return undistorted_img

    def srv_stop(self, req=None):

        undistorted_img = self.undistort(self.image_np)
        
        pos = self._detection_distance
        rate = rospy.Rate(self._calculation_rate)
        
        for i in range(100):

            start = rospy.get_rostime()
        
            # remove the image undesired section
            # height, width = self.image_np.shape[0:2]
            # image_stop = self.image_np[int(height*3/8) :, int(width/4):int(width*0.75), :]

            # hsv image
            imghsv = cv2.cvtColor(undistorted_img, cv2.COLOR_BGR2HSV)

            # HSV red color range
            red_lower_1 = np.array([0, 100, 20])
            red_upper_1 = np.array([10, 255, 255])
            red_lower_2 = np.array([160,100,20])
            red_upper_2 = np.array([179,255,255])

            mask1 = cv2.inRange(imghsv, red_lower_1, red_upper_1)
            mask2 = cv2.inRange(imghsv, red_lower_2, red_upper_2)
 
            red_mask = mask1 + mask2

            contours, hierachy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            if len(contours) != 0:
                c = max(contours, key = cv2.contourArea)
                hull = cv2.convexHull(c)
                x,y,w,h = cv2.boundingRect(c)

                M = cv2.moments(hull)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                else:
                    cX, cY = 0, 0

                bottom_px = 416 * (1 - 3/8) - cY

                pos = - 1.27e-07 * (bottom_px**3) + 3.618e-05 * (bottom_px**2) - 4.016e-03 * bottom_px - 4.694e-02

                # if self.stop:
                #     break

                # if pos > self._stop_distance:
                #     # pub FSM state to stop
                #     state = FSMState()
                #     state.state = "INTERSECTION_COORDINATION"
                #     self.pub_state.publish(state)
                #     time.sleep(1)
                #     self.stop = True


                if self.pub_dist.anybody_listening():

                    str_msg = str(pos)
                    self.pub_dist.publish(str_msg)

                #cv2.drawContours(image_stop, contours, -1, (0,255,0), 2)

            else:
                cX, cY = 0, 0

            if self.pub_center.anybody_listening():

                #image_center = cv2.circle(image_stop, (cX, cY), 3, (255, 0, 0), -1)
                image_center = cv2.rectangle(undistorted_img,(x,y),(x+w,y+h),(0,255,0),2)

                try:
                    self.pub_center.publish(self.bridge.cv2_to_compressed_imgmsg(image_center, "jpeg"))
                except CvBridgeError as e:
                    print(e)

            rate.sleep()

        rospy.loginfo("Duckiebot stopped at: %f m from stop line.", pos)

        response = DetectStopSignResponse()
        response.distance.data = pos

        end = rospy.get_rostime()
        time_t = (end - start).to_sec()
        rospy.loginfo("excecution time: %f", time_t)

        return response
    
    def camera(self, image):

        try:
            self.image_np = self.bridge.compressed_imgmsg_to_cv2(image, "bgr8")
            #self.image_np = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    # create the node
    node = StopSignDetector(node_name='stop_sign_detector_node')

    # keep spinning
    rospy.spin()
