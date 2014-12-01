robot-programming
=================

This is exercise for robot-programming.

```
$ source /opt/ros/hydro/setup.bash   
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws   
$ wstool init src
$ wstool set robot-programming https://github.com/jsk-enshu/robot-programming --git -t src -v 1.0.2
$ wstool update robot-programming -t src                                                                 
$ rosdep update                                                                                          
$ rosdep install --from-paths src  -y -r                                                                 
$ catkin_make                                                                                            
$ source ~/catkin_ws/devel/setup.bash ## 毎回ターミナルを開く度にこれを行うこと！！   
```

Documentations
=================
See online [manual](http://jsk-enshu.github.io/robot-programming/) for Euslisp models and interfaces.

PDF files are also available from [here](http://jsk-enshu.github.io/robot-programming/robot_programming_manual.pdf)

