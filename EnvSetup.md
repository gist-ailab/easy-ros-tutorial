# Environment Setup

- Ubuntu 18.04, ROS melodic

1. Install [ROS](http://wiki.ros.org/melodic/Installation/Ubuntu)
```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
$ sudo apt update
$ sudo apt install ros-melodic-desktop-full
$ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
$ echo "export ROS_MASTER_URI=http://localhost:11311" >> ~/.bashrc
$ echo "export ROS_HOSTNAME=localhost" >> ~/.bashrc
$ echo "export ROS_IP=localhost" >> ~/.bashrc
$ source ~/.bashrc
$ sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
```

2. Create [catkin workspace](http://wiki.ros.org/ko/catkin/Tutorials/create_a_workspace)
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
$ cd ~/catkin_ws/
$ catkin_make
```

3. Create [anaconda environment](https://github.com/Ankur-Deka/PyTorch-with-ROS-Installation-Guide)
```
$ conda create -n easyros python=2.7
$ conda activate easyros
$ conda install -c conda-forge rospkg catkin_pkg 
$ conda install pip
$ pip install -U rosinstall_generator wstool rosinstall six vcstools msgpack empy
```