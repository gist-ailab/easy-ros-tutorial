#!/usr/bin/env python

import rospy
import cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image

from easy_tcp_python2_3 import socket_utils as su


COCO_INSTANCE_CATEGORY_NAMES = [
    '__background__', 'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus',
    'train', 'truck', 'boat', 'traffic light', 'fire hydrant', 'N/A', 'stop sign',
    'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
    'elephant', 'bear', 'zebra', 'giraffe', 'N/A', 'backpack', 'umbrella', 'N/A', 'N/A',
    'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
    'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
    'bottle', 'N/A', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl',
    'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza',
    'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed', 'N/A', 'dining table',
    'N/A', 'N/A', 'toilet', 'N/A', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
    'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'N/A', 'book',
    'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush'
]

class ImageClassifier():

    def __init__(self):
        
        rospy.init_node("object_detector")
        rospy.loginfo("Starting object_detector node")
        self.cv_bridge = cv_bridge.CvBridge()

        # initialize dnn server 
        self.sock, add = su.initialize_server('localhost', 7777)
        rgb_sub = rospy.Subscriber("/rgb/image_raw", Image, self.callback)
        self.result_pub = rospy.Publisher("/detection_result", Image, queue_size=1)


    def callback(self, rgb):
        
        # convert ros imgmsg to opencv img
        rgb = self.cv_bridge.imgmsg_to_cv2(rgb, desired_encoding='bgr8')
        # send image to dnn client
        rospy.loginfo_once("send image to client")
        su.sendall_image(self.sock, rgb)
        rospy.loginfo_once("receive inference results from client")
        probs = su.recvall_pickle(self.sock)
        labels = su.recvall_pickle(self.sock)
        boxes = su.recvall_pickle(self.sock)
        masks = su.recvall_pickle(self.sock)

        for i in range(len(labels)):
            prob = probs[i]
            label = labels[i]
            x1, y1, x2, y2 = boxes[i].astype(int)
            mask = masks[i]
        
            cv2.putText(rgb, COCO_INSTANCE_CATEGORY_NAMES[int(label)], (x1+10, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 2, (0,0,255), 2)
            cv2.rectangle(rgb, (x1, y1), (x2, y2), (0,255,0), 3)


        # convert opencv img to ros imgmsg
        img_msg = self.cv_bridge.cv2_to_imgmsg(rgb, encoding='bgr8')
        
        # # publish it as topic
        self.result_pub.publish(img_msg)
        rospy.loginfo_once("Published the result as topic. check /detection_result")

    
if __name__ == '__main__':

    image_classificer = ImageClassifier()
    rospy.spin()