#!/usr/bin/env python3

import numpy as np
import rospy
import rospkg
import time
import math
import cv2
from cv_bridge import CvBridge, CvBridgeError
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, CameraInfo
from intersection_msgs.srv import DetectUsers, DetectUsersResponse
from std_msgs.msg import Float32
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
        self.fx = rospy.get_param('~fx')
        self.fy = rospy.get_param('~fy')
        self.cx = rospy.get_param('~cx')
        self.cy = rospy.get_param('~cy')
        self.k1 = rospy.get_param('~k1')
        self.k2 = rospy.get_param('~k2')
        self.k3 = rospy.get_param('~k3')
        self.k4 = rospy.get_param('~k4')
        self.duckie_h = rospy.get_param('~duckie_h')
        self.front_h = rospy.get_param('~front_h')
        self.back_h = rospy.get_param('~back_h')
        self.hfov = rospy.get_param('~hfov')
        self.img_w = rospy.get_param('~img_w')
        self.img_h = rospy.get_param('~img_h')

        self.interpreter = tflite.Interpreter(model_path=self.model_path)
        
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        self.height = self.input_details[0]['shape'][1]
        self.width = self.input_details[0]['shape'][2]

        self.boxes_idx, self.classes_idx, self.scores_idx = 1, 3, 0
        
        self.image_np = np.zeros((self.img_w, self.img_h, 3), dtype = "uint8")

        self.K = np.array([[self.fx,0.0,self.cx],[0.0,self.fy,self.cy],[0.0,0.0,1.0]])
        self.D=np.array([[self.k1],[self.k2],[self.k3],[self.k4]])

        self.focal = np.sqrt(np.power(self.fx, 2) + np.power(self.fy, 2)) / 2
        self.angle_constant = self.hfov / self.img_w

        # detect stop sign service
        self.srv_stop_sign = rospy.Service('~detect_users', DetectUsers, self.srv_detect)

        self.pub_time_ann = rospy.Publisher('~debug/inference_time', Float32, queue_size=1)

        self.pub_time_proj = rospy.Publisher('~debug/projection_time', Float32, queue_size=1)

        self.bridge = CvBridge()

        # subscriber to camera_node/image/compressed
        self.sub = rospy.Subscriber('/duckiebot4/camera_node/image/compressed', CompressedImage, self.camera, queue_size=1)

    def undistort(self, img):

        h, w = img.shape[0:2]
        newmatrix, roi = cv2.getOptimalNewCameraMatrix(self.K, self.D, (w,h), 1, (w,h))
        undistorted_img = cv2.undistort(img, self.K, self.D, None, newmatrix)
        x_r, y_r, w_r, h_r = roi
        undistorted_img = undistorted_img[y_r:y_r+h_r, x_r:x_r+w_r]
        resized_img = cv2.resize(undistorted_img, (self.img_w, self.img_h), cv2.INTER_NEAREST)

        return resized_img

    def coordinates(self, ymax, ymin, xmax, xmin, h):

        start_projection_time = rospy.get_rostime()

        distance = self.focal * h / (ymax - ymin)

        angle = ((self.hfov - 13) / 2) - self.angle_constant * ((xmax + xmin) / 2)
        angle_rads = np.radians(angle)

        pos_x = int(100 * distance * np.cos(angle_rads))
        pos_y = int(100 * distance * np.sin(angle_rads))

        end_projection_time = rospy.get_rostime()
        projection_time = (end_projection_time - start_projection_time).to_sec()

        return pos_x, pos_y, projection_time

    def srv_detect(self, req=None):

        undistorted_img = self.undistort(self.image_np)

        h, w = self.image_np.shape[0:2]

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

        if self.pub_time_ann.anybody_listening():

            time_msg = inference_time
            self.pub_time_ann.publish(time_msg)

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
                xmin = int(max(1,(boxes[i][1] * w)))
                ymax = int(min(h,(boxes[i][2] * h)))
                xmax = int(min(w,(boxes[i][3] * w)))

                object_name = int(classes[i])

                if(int(classes[i]) == 0):
                    pos_x, pos_y, projection_time = self.coordinates(ymax, ymin, xmax, xmin, self.back_h)

                elif(int(classes[i]) == 1):
                    pos_x, pos_y, projection_time = self.coordinates(ymax, ymin, xmax, xmin, self.duckie_h)

                else:
                    pos_x, pos_y, projection_time = self.coordinates(ymax, ymin, xmax, xmin, self.front_h)

                if self.pub_time_proj.anybody_listening():

                    time_msg_proj = projection_time
                    self.pub_time_proj.publish(time_msg_proj)

                out_score = int(100 * scores[i])

                detections.extend([object_name, out_score, pos_x, pos_y])

        out_buffer = DetectUsersResponse()
        out_buffer.detections.data = detections

        return out_buffer
    
    def camera(self, image):

        try:
            self.image_np = self.bridge.compressed_imgmsg_to_cv2(image)
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    # create the node
    node = ObjectDetector(node_name='detector_node')

    # keep spinning
    rospy.spin()