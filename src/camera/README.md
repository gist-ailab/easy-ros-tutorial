# Camera

Let's learn to play with Azure Kinect in ROS

1. Install Azure Kinect SDK [Link](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/docs/usage.md#Installation)
```
$ curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -
$ sudo apt-add-repository https://packages.microsoft.com/ubuntu/18.04/prod
$ sudo apt update
$ sudo apt install libk4a1.4 libk4a1.4-dev k4a-tools 
$ git clone https://github.com/microsoft/Azure-Kinect-Sensor-SDK
$ sudo cp Azure-Kinect-Sensor-SDK/scripts/99-k4a.rules /etc/udev/rules.d/
$ k4aviewer # usb 3 is required
```
2. Install [Azure Kinect ROS driver](https://github.com/microsoft/Azure_Kinect_ROS_Driver/blob/melodic/docs/building.md
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/microsoft/Azure_Kinect_ROS_Driver
$ cd ~/catkin_ws && catkin_make
```
3. Launch k4a driver
```
$ roslaunch azure_kinect_ros_driver driver.launch \
color_resolution:=720P depth_mode:=WFOV_BINNED fps:=5
```

4. RGB-D Capture

```
$ rosrun easy_ros_tutorial rgbd_capturer.py
```
<img src="../../imgs/rgbd_capture.png" height="150">


5. Color detection 
```
rosrun easy_ros_tutorial color_detector.py 
rosrun rqt_image_view rqt_image_view

```
<img src="../../imgs/color_detection.png" height="150">


6. Extrinsic calibration ([azure_kinect_manager](https://github.com/SeungBack/azure_kinect_manager))
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/SeungBack/azure_kinect_manager
$ ROS_NAMESPACE=azure1 roslaunch azure_kinect_ros_driver driver.launch color_resolution:=2160P depth_mode:=WFOV_2X2BINNED fps:=5 tf_prefix:=azure1_
$ roslaunch azure_kinect_manager single_azure_manager.launch 
$ rosservice call /azure1/get_camera_pose_single_markerboard "publish_worldmap: true
```
<img src="../../imgs/extrinsic_calibration.png" height="200">

7. Pointcloud processing

```
$ rosrun easy_ros_tutorial ponitcloud_processor.py 
```