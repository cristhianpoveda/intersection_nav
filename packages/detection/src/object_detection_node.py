#!/usr/bin/env python3

import numpy as np
import rospy
import rospkg
import time
import math
import cv2
from cv_bridge import CvBridge, CvBridgeError
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from std_srvs.srv import EmptyResponse, Empty
from intersection_msgs.srv import DetectUsers, DetectUsersResponse
import tflite_runtime.interpreter as tflite

class ObjectDetector(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(ObjectDetector, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        # params
        rospack = rospkg.RosPack()
        rospack.get_path('detection')
        self.model_path = rospack.get_path('detection') + rospy.get_param('~model_path')
        self.labels = rospy.get_param('~labels')

        self.interpreter = tflite.Interpreter(model_path=self.model_path)
        
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        self.height = self.input_details[0]['shape'][1]
        self.width = self.input_details[0]['shape'][2]
        
        self.image_np = np.zeros((640, 480, 3), dtype = "uint8")

        self.stop = False

        # detect stop sign service
        self.srv_stop_sign = rospy.Service('~detect_users', DetectUsers, self.srv_detect)

        self.bridge = CvBridge()

        # subscriber to camera_node/image/compressed
        self.sub = rospy.Subscriber('/duckiebot4/camera_node/image', Image, self.camera, queue_size=1)

    def srv_detect(self, req=None):

        # remove the image undesired section
        h, width = self.image_np.shape[0:2]
        input_img = self.image_np[int(h*1/4) :, :, :]

        # resize image to fit in the model
        dim = (self.width, self.height)
        input_img = cv2.resize(input_img, dim, interpolation = cv2.INTER_LINEAR)

        # add N dim
        input_data = np.expand_dims(input_img, axis=0)

        self.interpreter.set_tensor(self.input_details[0]['index'], input_data)

        # inference
        start = rospy.get_rostime()
        self.interpreter.invoke()
        end = rospy.get_rostime()
        inference_time = (end - start).to_sec()

        rospy.loginfo("time: %f", inference_time)

        output_data = self.interpreter.get_tensor(self.output_details[0]['index'])
        results = np.squeeze(output_data)

        rospy.loginfo("dims: %s", results.shape) 

        top_k = results.argsort()[-5:][::-1]

        rospy.loginfo("detections: %s", top_k)

        buffer_size = 2 * top_k.shape[0]

        buffer_array = np.zeros(buffer_size)

        for i in top_k:
            print('{:08.6f}: {}'.format(float(results[i] / 255.0), self.labels[i]))

        for i in range(top_k.shape[0]):
            buffer_array[2 * i] = top_k[i]
            buffer_array[i + 1] = float(results[i] / 255.0)

        detections = DetectUsersResponse()
        detections.detections.data = buffer_array

        return detections
    
    def camera(self, image):

        try:
            self.image_np = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    # create the node
    node = ObjectDetector(node_name='object_detection_node')

    # keep spinning
    rospy.spin()