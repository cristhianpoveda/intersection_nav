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

        self.K = np.array([[322.93673636507424,0.0,327.09788378259907],[0.0,321.852444071687,207.09902413788922],[0.0,0.0,1.0]])
        self.D=np.array([[-0.26239184522108644],[0.04732311174178657],[0.007784609968258371],[-0.0011570519649992076]])
        
        self.stop = False

        # detect stop sign service
        self.srv_stop_sign = rospy.Service('~detect_users', DetectUsers, self.srv_detect)

        self.bridge = CvBridge()

        # publisher (edited image)
        #self.pub_detections = rospy.Publisher('~detections', Image, queue_size=1)

        # subscriber to camera_node/image/compressed
        self.sub = rospy.Subscriber('/duckiebot4/camera_node/image', Image, self.camera, queue_size=1)

        #rospy.Timer(rospy.Duration(1.4), self.cb_timer)

    def undistort(self, img):
    
        h= img.shape[0]
        w = img.shape[1]
        newmatrix, roi = cv2.getOptimalNewCameraMatrix(self.K, self.D,(w,h),1, (w,h))
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(self.K, self.D, np.eye(3),self.K, (w,h), cv2.CV_32FC1)
        undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR)

        return undistorted_img

    def srv_detect(self, req=None):

        undistorted_img = self.undistort(self.image_np)

        # remove the image undesired section
        h, wi = self.image_np.shape[0:2]
        #input_img = self.image_np[int(h*1/4) :, :, :]

        # resize image to fit in the model
        dim = (self.width, self.height)
        resized_img = cv2.resize(undistorted_img, dim, interpolation = cv2.INTER_LINEAR)

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

        detections.append(len(scores))

        # Loop over all detections and draw detection box if confidence is above minimum threshold
        for i in range(len(scores)):
            if ((scores[i] > self.threshold) and (scores[i] <= 1.0)):

                # Get bounding box coordinates and draw box
                # Interpreter can return coordinates that are outside of image dimensions, need to force them to be within image using max() and min()
                ymin = int(max(1,(boxes[i][0] * h)))
                xmin = int(max(1,(boxes[i][1] * wi)))
                ymax = int(min(h,(boxes[i][2] * h)))
                xmax = int(min(wi,(boxes[i][3] * wi)))
            
                # cv2.rectangle(input_img, (xmin,ymin), (xmax,ymax), (10, 255, 0), 2)

                # # Draw label
                # object_name = self.labels[int(classes[i])] # Look up object name from "labels" array using class index
                object_name = int(classes[i])
                # label = '%s: %d%%' % (object_name, int(scores[i]*100)) # Example: 'person: 72%'
                # labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2) # Get font size
                # label_ymin = max(ymin, labelSize[1] + 10) # Make sure not to draw label too close to top of window
                # cv2.rectangle(input_img, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), (255, 255, 255), cv2.FILLED) # Draw white box to put label text in
                # cv2.putText(input_img, label, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2) # Draw label text

                y_proj = int(100 * (ymax - ymin))
                x_proj = int(100 * (xmax - xmin))

                out_score = int(100 * scores[i])
                detections.extend([object_name, out_score, x_proj, y_proj])

        rospy.loginfo(detections)

        out_buffer = DetectUsersResponse()
        out_buffer.detections.data = detections

        return out_buffer
    
    def camera(self, image):

        try:
            self.image_np = self.compressed_imgmsg_to_cv2(image)
            #self.image_np = self.bridge.imgmsg_to_cv2(image, "rgb8")
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    # create the node
    node = ObjectDetector(node_name='detector_node')

    # keep spinning
    rospy.spin()