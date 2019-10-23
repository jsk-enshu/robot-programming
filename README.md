robot-programming [![Build Status](https://travis-ci.org/jsk-enshu/robot-programming.svg?branch=master)](https://travis-ci.org/jsk-enshu/robot-programming) [![Join the chat at https://gitter.im/jsk-enshu/robot-programming](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/jsk-enshu/robot-programming?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)
=================

This is exercise for robot-programming.

Setup
-----

```
$ source /opt/ros/indigo/setup.bash   
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone https://github.com/jsk-enshu/robot-programming
$ wstool init .
$ wstool merge robot-programming/.rosinstall.${ROS_DISTRO}
$ wstool update
$ rosdep update
$ rosdep install --from-paths src --ignore-src -y -r
$ cd ..
$ catkin build
# $ echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc ## > と >> の違いが理解できていればbashrcに追加してもよい
```

Start simulator
---------------
```
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch dxl_armed_turtlebot dxl_armed_turtlebot_gazebo.launch
```

Start RQT GUI
-------------
```
$ roscd dxl_armed_turtbot/launch
$ rqt --perspective-file enshu.perspective
```

Start Color Tracking node
-------------------------
```
$ roslaunch opencv_apps camshift.launch image:=/camera/rgb/image_raw
$ rosrun image_view2 image_view2 image:=/camera/rgb/image_raw ~image_transport:=compressed
```

Start Checkerboard Tracking Tracking node
-----------------------------------------
```
$ roslaunch checkerboard-detector.launch rect0_size_x:=0.02 rect0_size_y:=0.02
      grid0_size_x:=7 grid0_size_y:=4 translation0:="0 0 0"
      image:=image_raw  group:=/camera/rgb frame_id:=camera_rgb_optical_frame
$ ROS_NAMESPACE=/camera/rgb rosrun checkerard_detector objectdetection_tf_publisher.py
       _use_simple_tf:=true
```

Documentations
=================
See online [manual](http://jsk-enshu.github.io/robot-programming/) for Euslisp models and interfaces.

PDF files are also available from [here](http://jsk-enshu.github.io/robot-programming/robot_programming_manual.pdf)

