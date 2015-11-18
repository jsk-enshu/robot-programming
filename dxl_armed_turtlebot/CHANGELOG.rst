^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dxl_armed_turtlebot
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.2 (2015-11-12)
------------------
* Use kinect as 3d_sensor
* Contributors: Shunichi Nozawa

2.1.1 (2015-11-11)
------------------
* add robot_state_publisher for both base and robot
* Contributors: Kei Okada

2.1.0 (2015-11-11)
------------------
* add dxl_armed_turtlebot_gazebo.launch
* add dxl_armed_turtlebot/urdf/robot.urdf.xacro
* add depends to dynamixel_urdf (update .travis.yml, README.md and dxl_armed_turtlebot/package.xml
  )
* Contributors: Kei Okada

2.0.0 (2015-11-10)
------------------
* dxl_armed_turtlebot, dynamixel_7dof_arm, turtleboteus: add roseus tofind_package() to  gen messages
* Contributors: Kei Okada

1.0.3 (2015-11-09)
------------------
* Enable to create several hsi filters
* Fix camera frame (rgb frame)
* Contributors: Shunichi Nozawa

1.0.2 (2014-12-01)
------------------
* Add launch file includes all enshu launches
* Update html encoding
* Add documentation directory for robot_programming all
* Use require instead of load
* Eval generated defmethod outside of :init
* Add vision example from jsk_pcl_ros/euslisp/display-bounding-box-array.l
* Contributors: Shunichi Nozawa

1.0.1 (2014-11-27)
------------------
* Remove unused method :def-vector-value
* Remove unused models which are moved to turtleboteus package
* Add documentation for Euslisp codes
* Add hsi_color_filter tempolarily.
* Add turtleboteus rostest and update other tests
* Add turtleboteus package and use it from dxl-armed-turtlebot
* receive rest arguments in update-robot-staet
* Contributors: Shunichi Nozawa, mmurooka

1.0.0 (2014-11-11)
------------------
* add rostest to package.xml
* add control_msgs and move_base_msgs
* add pr2eus to depends
* Revert https://github.com/jsk-enshu/robot-programming/commit/8eda7005768bae4a1c3783fe5d975f551501a42a
* Add tests for dxl_armed_turtlebot and dynamixel_7dof_arm
* Update :update-robot-state according to latest argument
* remove linux_hardware from find_package, which is not supported on indigo
* remove rosbuild code
* Move robot-programming enshu packages from source forge repository
* Contributors: Kei Okada, Shunichi Nozawa
