^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package turtleboteus
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.0.0 (2017-09-24)
------------------
* enable to run travis on kinetic (`#171 <https://github.com/jsk-enshu/robot-programming/issues/171>`_)
  * smart_battery_msgs is obsolete in kinetic, use sensor_msgs/BatteryState
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

2.1.2 (2015-11-12)
------------------

2.1.1 (2015-11-11)
------------------

2.1.0 (2015-11-11)
------------------

2.0.0 (2015-11-10)
------------------
* linux_hardware is obsoulte, use  linux_peripheral_interface or smart_battery_msgs
* ;; turtlebot-interface-common.l; I'm not sure but without this change, the knaji code crashes
* dxl_armed_turtlebot, dynamixel_7dof_arm, turtleboteus: add roseus tofind_package() to  gen messages
* {daisya_euslisp_tutorials,turtleboteus}/{package.xml,CMakeLists.txt}: add rostest to {build,run}_depend
* turtleboteus/package.xml: linux_hardware is now move to linux_peripheral_interface and smart_battery_msgs
* {turtleboteus,dynamixel_7dof_arm}/package.xml roseus_msgs is not longer required
* Contributors: Kei Okada

1.0.3 (2015-11-09)
------------------

1.0.2 (2014-12-01)
------------------
* Add go-pos methods documentation
* Use require instead of load
* Eval generated defmethod outside of :init
* Contributors: Shunichi Nozawa

1.0.1 (2014-11-27)
------------------
* Fix view-up orientation, which is reported in https://github.com/jsk-enshu/robot-programming/issues/23
* Move simulation check to turtlebot-interface.l
* Fix cameras and bumper-sensors accessors
* Add documentation for Euslisp codes
* Add turtleboteus rostest and update other tests
* Add turtleboteus package and use it from dxl-armed-turtlebot
* Contributors: Shunichi Nozawa
