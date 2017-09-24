^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dxl_armed_turtlebot
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.0.0 (2017-09-24)
------------------
* fix for latest kinetic / gazebo (`#172 <https://github.com/jsk-enshu/robot-programming/issues/172>`_)
  * fix enshu.perspective for kinetic
  * update kobuki urdf location for kinetic
* enable to run travis on kinetic (`#171 <https://github.com/jsk-enshu/robot-programming/issues/171>`_)
  * change urdf path
* add dxl-arm-sample function (`#170 <https://github.com/jsk-enshu/robot-programming/issues/170>`_, `#160 <https://github.com/jsk-enshu/robot-programming/issues/160>`_)
  * define function dxl-arm-check to excute
  * update arm-move-sample
  * add arm-move-sample.l
* Contributors: Kei Okada, Yuki Asano

3.2.2 (2016-12-01)
------------------
* dxl_armed_turtlebot_navigation.launch: run velocity_smoother_renamer (topic_tools/transform) with --wait-for-start (`#151 <https://github.com/jsk-enshu/robot-programming/issues/151>`_)
* Contributors: Naoya Yamaguchi

3.2.1 (2016-11-30)
------------------
* change turtlebot arm direction up side down
* add kobuki_dashboard to run depend
* remove duplicated Contributor
* Contributors: Yuki Asano

3.2.0 (2016-11-29)
------------------
* add slam_karto to run depend
* fix to publish appropriate topic
* add depend pkg
* change default arm mounting direction
* add depend pkgs
* modify to print
* add joy-sample.l for document simplification
* Merge pull request #134 from k-okada/fix_ros_apps
  add map_server to package.xml
* fix amcl.launch location for turtlebot_navigation 2.3.7
* add map_server
* Contributors: Kei Okada, Yuki Asano

3.1.0 (2016-11-07)
------------------
* add depthimage_to_laserscan to package.xml
* add dxl_armed_turtlebot_navigation.launch
* add jsk_recognition packages to depend
* add depth_to_laserfilter
* add walls for gazebo world
* add more displays, path, arrow
* Contributors: Kei Okada

3.0.1 (2016-11-06)
------------------
* add missing deps
* Contributors: Taiki Abe

3.0.0 (2016-10-16)
------------------
* update for 2016 (https://github.com/jsk-enshu/robot-programming/pull/78)

  * package.xml : add image_view2 to depends
  * package.xml : add depends to opencv_apps
  * package.xml : more package depends
  * package.xml :  add nodelet to package.xml
  * package.xml : add turtlebot_teleop to package.xml

  * add enshu.perspective and update package.xml
  * CMakeLists.txt : clean up roslaunch_add_file_check
  * update gazebo parametesrs, use velocity_controller, initialie robot pose by turk_arm.py
  * move controller_config to config/config/dxl_armed_turtlebot_controller.yaml
  * use dxl_armed_turtlebot.rviz settings
  * set ros_control namespace from /dxl_armed_turtlebot to /, and robot_state_publisher taks joint_states from both base and arm and publish tf
  * use custom worlds/empty.world, for simulation step at 0.01
  * urdf/dynamixel_7dof_arm.urdf.xacro: fix typo Gray -> Grey
  * dxl_armed_turtlebot/{package.xml, CMakeLists.txt}

* dxl_armed_turtlebot/launch/hsi_color_filter.launch: add hue, satuation, intensity parameter information #73 (https://github.com/jsk-enshu/robot-programming/pull/73)

  * modified satuation information
  * add s, i param information
  * add hue parameter information (-128~127 = -p1~pi)

* Contributors: Kanae Kochigami, Kei Okada

2.1.5 (2015-11-25)
------------------

2.1.4 (2015-11-24)
------------------
* Update hsi color for latest jsk_pcl_ros and enable to invoke several color filter
* Update comments for display euslisp examples
* Contributors: Shunichi Nozawa

2.1.3 (2015-11-19)
------------------
* Update arm base
* Add example to display checkerboard pose with turtlebot
* Fix ps3 teleop launch path
* Update ps3joy teleop launch to reduce velocity and use indigo
* Update displaying of bounding box using jsk_pcl_ros. Update package name and topic names.
* Contributors: Shunichi Nozawa

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
