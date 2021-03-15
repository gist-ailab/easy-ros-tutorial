#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from open3d_ros_helper import open3d_ros_helper as orh

class PointcloudProcessor():

    def __init__(self):
        
        rospy.init_node("pointcloud_processor")
        rospy.loginfo("Starting pointcloud_processor node")
        point_sub = rospy.Subscriber("/points2", PointCloud2, self.callback)
        self.point_pub = rospy.Publisher("/filt_point2", PointCloud2, queue_size=1)
        
        self.voxel_size = 0.01 # 1 cm
        self.ROI = {'x': [-0.2, 0.2], 
                    'y': [-0.2, 0.2],
                    'z': [-1.0, 1.0]} 

    def callback(self, msg):
        
        # convert ros imgmsg to opencv img
        cloud = orh.rospc_to_o3dpc(msg, remove_nans=True)
        cloud = cloud.voxel_down_sample(voxel_size=self.voxel_size)
        cloud = orh.apply_pass_through_filter(cloud, self.ROI['x'], self.ROI['y'], self.ROI['z'])
        cloud = orh.o3dpc_to_rospc(cloud, frame_id=msg.header.frame_id)
        self.point_pub.publish(cloud)
        
        rospy.loginfo_once("Published the result as topic. check /filt_point2")

    
if __name__ == '__main__':

    pointcloud_processor = PointcloudProcessor()
    rospy.spin()