^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dynamixel_7dof_arm
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

6.1.0 (2019-11-11)
------------------
* update dxl-7dof-arm-robot's joint and pose method (`#291 <https://github.com/jsk-enshu/robot-programming/issues/291>`_)
* Contributors: Kei Okada, Masaki Murooka

6.0.0 (2019-06-16)
------------------
* fix for melodic and add test on travis (`#272 <https://github.com/jsk-enshu/robot-programming/issues/272>`_)

  * fix xacro: deprecated: xacro tags should be prepended with 'xacro' xml namespace.
    Use the following script to fix incorrect usage:
    find . -iname *.xacro | xargs sed -i 's#<\([/]\?\)\(if\|unless\|include\|arg\|property\|macro\|insert_block\)#<\1xacro:\2#g'
    when processing file: /home/k-okada/catkin_ws/ws_enshu/src/robot-programming/dynamixel_7dof_arm/urdf/dynamixel_7dof_arm.urdf.xacro
  * fix: The STL file 'package://dynamixel_7dof_arm/urdf/meshes/dxl_gripper.stl' is malformed. It starts with the word 'solid', indicating that it's an ASCII STL file, but it does not contain the word 'endsolid' so it is either a malformed ASCII STL file or it is actually a binary STL file. Trying to interpret it as a binary STL file instead.
  * fix xacro : property -> xacro:property
  * set joint_interface to hardware_interface/VelocityJointInterface
  * run test/*.test only when pr2eus installed
  * we do not need roseus/dynamixel_controllers for compile

* Armed turtlebot in  Gazebo (`#245 <https://github.com/jsk-enshu/robot-programming/issues/245>`_)

  * Update the urdf model for dxynamixel 7dof arm to match the true robot model
  * Add the callback function to get joint_states from gazebo model in dxl-7dof-arm robot-interface.
  * Change the joint name form "arm_link${num}_joint" to "arm_joint${num}", in order to match the naming rule in real robot and eus model
  * Add friction rate for gripper and object, along with PID gains for girpper controller
  * Update the ros related function for gripper joint (e.g., actionlib, subscriber, rosservice).
    Check with both roseus keinamtic simulator and real robot.

* Update the robot model of dxl_7dof_arm (`#244 <https://github.com/jsk-enshu/robot-programming/issues/244>`_)

  * Change the controller names for gripper joint (arm_j7 => gripper_controller)
  * Update the robot model of dxl_7dof_arm, seperating arm actuators and gripper
* Contributors: Kei Okada, Moju Zhao

5.0.0 (2018-09-14)
------------------
* Add gripper urdf model for rviz and gazebo (`#239 <https://github.com/jsk-enshu/robot-programming/pull/239>`_)
* Add ros dependency explicitly in package.xml (`#243 <https://github.com/jsk-enshu/robot-programming/pull/243>`_)
* Contributors: Bakui Chou

4.0.1 (2017-11-20)
------------------

4.0.0 (2017-09-24)
------------------
* enable to run travis on kinetic (`#171 <https://github.com/jsk-enshu/robot-programming/issues/171>`_)
  * update msg load description
  * add depends and load manifest
* Contributors: Yuki Asano

3.2.2 (2016-12-01)
------------------

3.2.1 (2016-11-30)
------------------

3.2.0 (2016-11-29)
------------------

3.1.0 (2016-11-07)
------------------

3.0.1 (2016-11-06)
------------------

3.0.0 (2016-10-16)
------------------

2.1.5 (2015-11-25)
------------------

2.1.4 (2015-11-24)
------------------

2.1.3 (2015-11-19)
------------------
* Add arm pose to get wide view field
* Contributors: Shunichi Nozawa

2.1.2 (2015-11-12)
------------------

2.1.1 (2015-11-11)
------------------

2.1.0 (2015-11-11)
------------------

2.0.0 (2015-11-10)
------------------
* dxl_armed_turtlebot, dynamixel_7dof_arm, turtleboteus: add roseus tofind_package() to  gen messages
* {turtleboteus,dynamixel_7dof_arm}/package.xml roseus_msgs is not longer required
* Contributors: Kei Okada

1.0.3 (2015-11-09)
------------------
* Comment out speak-jp because currently aques_talk is not available
* Contributors: Shunichi Nozawa

1.0.2 (2014-12-01)
------------------
* Add documentation for arm robot
* Use require instead of load
* Eval generated defmethod outside of :init
* Contributors: Shunichi Nozawa

1.0.1 (2014-11-27)
------------------
* Add documentation for Euslisp codes
* Add turtleboteus rostest and update other tests
* Contributors: Shunichi Nozawa

1.0.0 (2014-11-11)
------------------
* Add tests for dxl_armed_turtlebot and dynamixel_7dof_arm
* Add dependency on pr2eus and roseus_msgs
* remove rosbuild code
* Move robot-programming enshu packages from source forge repository
* Contributors: Kei Okada, Shunichi Nozawa
