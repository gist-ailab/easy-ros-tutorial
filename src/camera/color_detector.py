#!/usr/bin/env python

import rospy
import cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image

class ColorDetector():

    def __init__(self):
        
        rospy.init_node("color_detector")
        rospy.loginfo("Starting color_detector node")
        rgb_sub = rospy.Subscriber("/rgb/image_raw", Image, self.callback)
        self.cv_bridge = cv_bridge.CvBridge()
        self.result_pub = rospy.Publisher("/color_detect/image_raw", Image, queue_size=1)

        self.lower_bound = np.array([155, 25, 0]) # hsv
        self.upper_bound = np.array([179, 255, 255]) # hsv


    def callback(self, rgb):
        
        # convert ros imgmsg to opencv img
        rgb = self.cv_bridge.imgmsg_to_cv2(rgb, desired_encoding='bgr8')
        
        # red region detection 
        rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(rgb, self.lower_bound, self.upper_bound)
        result = cv2.bitwise_and(rgb, rgb, mask=mask)
        result = cv2.cvtColor(result, cv2.COLOR_HSV2BGR)

        # convert opencv img to ros imgmsg
        img_msg = self.cv_bridge.cv2_to_imgmsg(result, encoding='bgr8')
        
        # publish it as topic
        self.result_pub.publish(img_msg)
        rospy.loginfo_once("Published the result as topic. check '/color_detect'/image_raw")

    
if __name__ == '__main__':

    color_detector = ColorDetector()
    rospy.spin()