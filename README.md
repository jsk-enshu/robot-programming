robot-programming [![Build Status](https://app.travis-ci.com/jsk-enshu/robot-programming.svg?branch=master)](https://app.travis-ci.com/jsk-enshu/robot-programming) [![Join the chat at https://gitter.im/jsk-enshu/robot-programming](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/jsk-enshu/robot-programming?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)
=================

This is exercise for robot-programming.

# Setup
-----

```
$ sudo apt install python3-vcstool
$ source /opt/ros/noetic/setup.bash
$ mkdir -p ~/ros/enshu_ws/src
$ cd ~/ros/enshu_ws/src
$ wget https://raw.githubusercontent.com/iory/robot-programming/refs/heads/kx-pr2/vcsinstall.noetic.yaml -O- | vcs import
$ rosdep update
$ cd ~/ros/enshu_ws
$ rosdep install --from-paths src --ignore-src -y -r
$ catkin build
# $ echo 'source ~/ros/enshu_ws/devel/setup.bash' >> ~/.bashrc ## You may source the setup.bash by this line if you understand the difference between > and >>.
```

# KX-PR2 simulation
## Start simulator
---------------
```
$ source ~/ros/enshu_ws/devel/setup.bash
$ roslaunch kx_pr2_bringup kx_pr2_gazebo.launch
```

## Start RQT GUI
-------------
```
$ roscd kx_pr2_bringup/launch
$ rqt --perspective-file enshu.perspective
```

## Start Color Tracking node
-------------------------
```
$ roslaunch opencv_apps camshift.launch image:=/camera/rgb/image_raw
$ rosrun image_view2 image_view2 image:=/camera/rgb/image_raw ~image_transport:=compressed
```

## Start Checkerboard Tracking Tracking node
-----------------------------------------
```
$ roslaunch roseus_tutorials checkerboard-detector.launch rect0_size_x:=0.02 rect0_size_y:=0.02 \
      grid0_size_x:=7 grid0_size_y:=4 translation0:="0 0 0" \
      image:=image_raw  group:=/camera/rgb frame_id:=camera_rgb_optical_frame
$ ROS_NAMESPACE=/camera/rgb rosrun checkerboard_detector objectdetection_tf_publisher.py \
       _use_simple_tf:=true
```

# cart_humanoid (JAXON) simulation
![cart humanoid](./cart_humanoid/images/cart_humanoid_gazebo.png)

## Start simulator
---------------
```
$ source ~/ros/enshu_ws/devel/setup.bash
$ roslaunch cart_humanoid cart_humanoid_gazebo.launch
# It may take a few minutes to start the simulator for the first time.
# If you launch simulators with the robot's base link unfixed, use the following command instead.
$ roslaunch cart_humanoid cart_humanoid_gazebo.launch fix_base_link:=false
```

## Start RQT GUI
-------------
```
$ rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```

# Documentations
----------------


See online [manual](http://jsk-enshu.github.io/robot-programming/) for Euslisp models and interfaces.

PDF files are also available from [here](http://jsk-enshu.github.io/robot-programming/robot_programming_manual.pdf)

