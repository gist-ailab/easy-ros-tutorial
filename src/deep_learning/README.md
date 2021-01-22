# Deep Learning

- Let's learn to play with Azure Kinect in ROS

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
1. Run DNN server node (ROS, python 2.7)
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

<img src="../../imgs/image_classification.png" height="200">


## Object Detection
1. Run DNN server node (ROS, python 2.7)
```
$ conda activate easyros
$ rosrun easy_ros_tutorial object_detector.py
```
3. Run DNN client node (ROS, python 3.7)
```
$ conda activate easyros-py37
$ roscd easy_ros_tutorial && python src/deep_learning/object_detector_client.py
```

<img src="../../imgs/object_detection.png" height="200">



