^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package daisya_euslisp_tutorials
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.0.0 (2017-09-24)
------------------

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
* add :arm-end-coords method, this will solve error, cannot find method :arm-end-coords in (apply #'send self :limb :arm args)
* Contributors: Kei Okada

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
* daisya_euslisp_tutorials: add depends to dxl_armed_turtlebot
* {daisya_euslisp_tutorials,turtleboteus}/{package.xml,CMakeLists.txt}: add rostest to {build,run}_depend
* Contributors: Kei Okada

1.0.3 (2015-11-09)
------------------

1.0.2 (2014-12-01)
------------------

1.0.1 (2014-11-27)
------------------
* test-subsumption should run more than 100, since the sensor data changes at i = 100
* Contributors: Kei Okada

1.0.0 (2014-11-11)
------------------
* Fix final ik for reaching
* Add find_package for rostest
* Enable to catkin_make test for several examples
* remove rosbuild code
* Move robot-programming enshu packages from source forge repository
* Contributors: Kei Okada, Shunichi Nozawa
