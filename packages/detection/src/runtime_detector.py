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
        self.threshold = rospy.get_param('~threshold')

        self.interpreter = tflite.Interpreter(model_path=self.model_path)
        
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        self.height = self.input_details[0]['shape'][1]
        self.width = self.input_details[0]['shape'][2]

        self.boxes_idx, self.classes_idx, self.scores_idx = 1, 3, 0
        
        self.image_np = np.zeros((640, 480, 3), dtype = "uint8")

        self.stop = False

        # detect stop sign service
        #self.srv_stop_sign = rospy.Service('~detect_users', DetectUsers, self.srv_detect)

        self.bridge = CvBridge()

        # publisher (edited image)
        self.pub_detections = rospy.Publisher('~detections', Image, queue_size=1)

        # subscriber to camera_node/image/compressed
        self.sub = rospy.Subscriber('/duckiebot4/camera_node/image', Image, self.camera, queue_size=1)

        rospy.Timer(rospy.Duration(1.4), self.cb_timer)

    def cb_timer(self, event): #req=None

        # remove the image undesired section
        h, wi = self.image_np.shape[0:2]
        input_img = self.image_np[int(h*1/4) :, :, :]

        # resize image to fit in the model
        dim = (self.width, self.height)
        resized_img = cv2.resize(input_img, dim, interpolation = cv2.INTER_LINEAR)

        # add N dim
        input_data = np.expand_dims(resized_img, axis=0)

        self.interpreter.set_tensor(self.input_details[0]['index'], input_data)

        # inference
        start = rospy.get_rostime()
        self.interpreter.invoke()
        end = rospy.get_rostime()
        inference_time = (end - start).to_sec()

        rospy.loginfo("Inference time: %f", inference_time)

        # Retrieve detection results
        boxes = self.interpreter.get_tensor(self.output_details[self.boxes_idx]['index'])[0] # Bounding box coordinates of detected objects
        classes = self.interpreter.get_tensor(self.output_details[self.classes_idx]['index'])[0] # Class index of detected objects
        scores = self.interpreter.get_tensor(self.output_details[self.scores_idx]['index'])[0] # Confidence of detected objects

        detections = []

        # Loop over all detections and draw detection box if confidence is above minimum threshold
        for i in range(len(scores)):
            if ((scores[i] > self.threshold) and (scores[i] <= 1.0)):

                # Get bounding box coordinates and draw box
                # Interpreter can return coordinates that are outside of image dimensions, need to force them to be within image using max() and min()
                ymin = int(max(1,(boxes[i][0] * h)))
                xmin = int(max(1,(boxes[i][1] * wi)))
                ymax = int(min(h,(boxes[i][2] * h)))
                xmax = int(min(wi,(boxes[i][3] * wi)))
            
                cv2.rectangle(input_img, (xmin,ymin), (xmax,ymax), (10, 255, 0), 2)

                # # Draw label
                object_name = self.labels[int(classes[i])] # Look up object name from "labels" array using class index
                label = '%s: %d%%' % (object_name, int(scores[i]*100)) # Example: 'person: 72%'
                labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2) # Get font size
                label_ymin = max(ymin, labelSize[1] + 10) # Make sure not to draw label too close to top of window
                cv2.rectangle(input_img, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), (255, 255, 255), cv2.FILLED) # Draw white box to put label text in
                cv2.putText(input_img, label, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2) # Draw label text

                detections.append([object_name, scores[i], xmin, ymin, xmax, ymax])

        try:
            self.pub_detections.publish(self.bridge.cv2_to_imgmsg(input_img, "rgb8"))
        except CvBridgeError as e:
            print(e)

        #print(detections)

        out_buffer = DetectUsersResponse()
        out_buffer.detections.data = [1,2]

        return out_buffer
    
    def camera(self, image):

        try:
            self.image_np = self.bridge.imgmsg_to_cv2(image, "rgb8")
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    # create the node
    node = ObjectDetector(node_name='runtime_detector')

    # keep spinning
    rospy.spin()