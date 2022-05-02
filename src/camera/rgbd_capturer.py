#!/usr/bin/env python

import rospy
import cv2, cv_bridge
import numpy as np
import message_filters
from sensor_msgs.msg import Image, PointCloud2
import os
import open3d as o3d
from open3d_ros_helper import open3d_ros_helper as orh

    
if __name__ == '__main__':


    rospy.init_node("rgbd_capturer")
    rospy.loginfo("Starting rgbd_capturer node")
    bridge = cv_bridge.CvBridge()
    
    img_id = 0
    save_root = "./data"
    min_depth, max_depth = 250, 3500
    print("min_depth: " + str(min_depth) + "  max_depth: " + str(max_depth))

    while True:
        
        # get rgb-depth from camera
        rgb_msg = rospy.wait_for_message("/rgb/image_raw", Image)
        depth_msg = rospy.wait_for_message("/depth_to_rgb/image_raw", Image)
        pc_msg = rospy.wait_for_message("/points2", PointCloud2)
        rgb = bridge.imgmsg_to_cv2(rgb_msg,desired_encoding='bgr8')
        depth = bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')

        # depth normalization (0.25 ~ 1.25 -> 0 ~ 255)
        depth_im = depth
        depth_im[depth_im > max_depth] = max_depth
        depth_im[depth_im < min_depth] = min_depth
        depth_im = np.uint8(255 * (depth_im - min_depth) / (max_depth - min_depth))
        depth_im = np.repeat(np.expand_dims(depth_im, -1), 3, -1)
        rgbd_im = np.hstack([rgb, depth_im])
        rgbd_im = cv2.resize(rgbd_im, (1280, 360))
        cv2.imshow("rgbd - img_id: " + str(img_id) + "   S: Save / Q: Quit", rgbd_im )

        key = cv2.waitKey(1)
        if key == ord('q'):
            break

        elif key == ord('s'):
            print("Saving RGB-Depth, Image ID=" + str(img_id) + "  SAVE_ROOT=" + save_root)
            cv2.imwrite(os.path.join(save_root, "rgb_{}.png".format(img_id)), rgb)
            cv2.imwrite(os.path.join(save_root, "depth_im_{}.png".format(img_id)), depth_im)
            np.save(os.path.join(save_root, "depth_{}.npy".format(img_id)), depth)
            o3dpc = orh.rospc_to_o3dpc(pc_msg) 
            o3d.io.write_point_cloud(os.path.join(save_root, "o3d_cloud_{}.ply".format(img_id)), o3dpc)
            o3d.visualization.draw_geometries([o3dpc])
            img_id += 1
            cv2.destroyAllWindows()
            





