# Deep Learning

Let's learn to play wit deep learning in ROS

Setup anaconda environment and install dependencies
```
# python 2
$ conda create -n easyros python=2.7
$ conda activate easyros 
$ easy-tcp-python2-3 pickle-compat
# python 3
$ conda create -n easyros-py37 python=3.7
$ conda activate easyros-py37
$ pip install torch torchvision easy-tcp-python2-3 pickle-compat opencv-python six
$ pip install git+https://github.com/nottombrown/imagenet_stubs
```

## Image Classification

<img src="../../imgs/image_classification.png" height="200">

1. Launch k4a driver
```
$ roslaunch azure_kinect_ros_driver driver.launch \
color_resolution:=720P depth_mode:=NFOV_2X2BINNED fps:=5
```

2. Run DNN server node (ROS, python 2.7)
```
$ conda activate easyros
$ rosrun easy_ros_tutorial img_classifier.py
```

3. Run DNN client node (ROS, python 3.7)
```
# ROS1 does not support python 3.7 stably, while DNN often requires python 3.7
# We use tcp socket to communicate with DNN client
# ROS1 server ==(image)=> DNN client ==(inference result)==> ROS1 server

$ conda activate easyros-py37
$ roscd easy_ros_tutorial && python src/deep_learning/img_classifier_client.py
```



## Object Detection

<img src="../../imgs/object_detection.png" height="200">

1. Launch k4a driver
```
$ roslaunch azure_kinect_ros_driver driver.launch \
color_resolution:=720P depth_mode:=NFOV_2X2BINNED fps:=5
```

2. Run DNN server node (ROS, python 2.7)
```
$ conda activate easyros
$ rosrun easy_ros_tutorial object_detector.py
```

3. Run DNN client node (ROS, python 3.7)
```
$ conda activate easyros-py37
$ roscd easy_ros_tutorial && python src/deep_learning/object_detector_client.py
```




