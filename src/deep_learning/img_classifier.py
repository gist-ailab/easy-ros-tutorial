#!/usr/bin/env python

import rospy
import cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image

from easy_tcp_python2_3 import socket_utils as su


class ImageClassifier():

    def __init__(self):
        
        rospy.init_node("image_classifier")
        rospy.loginfo("Starting image_classifier node")
        self.cv_bridge = cv_bridge.CvBridge()

        # initialize dnn server 
        self.sock, add = su.initialize_server('localhost', 7777)
        rgb_sub = rospy.Subscriber("/rgb/image_raw", Image, self.callback)
        self.result_pub = rospy.Publisher("/classification_result", Image, queue_size=1)

    def callback(self, rgb):
        
        # convert ros imgmsg to opencv img
        rgb = self.cv_bridge.imgmsg_to_cv2(rgb, desired_encoding='bgr8')
        H, W, C = rgb.shape
        rgb_crop = rgb[int(H/4):int(3*H/4), int(W/4):int(3*W/4), :]
        rgb_crop = cv2.resize(rgb_crop, (224, 224))
        
        # send image to dnn client
        rospy.loginfo_once("send image to client")
        su.sendall_image(self.sock, rgb_crop)
        rospy.loginfo_once("receive inference results from client")
        class_name = su.recvall_pickle(self.sock)
        cv2.putText(rgb, class_name, (int(0.4*W), int(0.9*H)), cv2.FONT_HERSHEY_SIMPLEX, 2, (0,0,255), 2)
        cv2.rectangle(rgb, (int(W/4), int(H/4)), (int(3*W/4), int(3*H/4)), (0,255,0), 3)


        # convert opencv img to ros imgmsg
        img_msg = self.cv_bridge.cv2_to_imgmsg(rgb, encoding='bgr8')
        
        # # publish it as topic
        self.result_pub.publish(img_msg)
        rospy.loginfo_once("Published the result as topic. check /classification_result")

    
if __name__ == '__main__':

    image_classificer = ImageClassifier()
    rospy.spin()