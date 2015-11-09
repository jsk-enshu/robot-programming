robot-programming[![Build Status](https://travis-ci.org/jsk-enshu/robot-programming.svg?branch=master)](https://travis-ci.org/jsk-enshu/robot-programming)
=================

This is exercise for robot-programming.

```
$ source /opt/ros/indigo/setup.bash   
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws   
$ wstool init src
$ wstool set robot-programming https://github.com/jsk-enshu/robot-programming --git -t src
$ wstool update robot-programming -t src                                                                 
$ rosdep update                                                                                          
$ rosdep install --from-paths src  -y -r                                                                 
$ catkin build
$ source ~/catkin_ws/devel/setup.bash ## 毎回ターミナルを開く度にこれを行うこと！！   
```

Documentations
=================
See online [manual](http://jsk-enshu.github.io/robot-programming/) for Euslisp models and interfaces.

PDF files are also available from [here](http://jsk-enshu.github.io/robot-programming/robot_programming_manual.pdf)

