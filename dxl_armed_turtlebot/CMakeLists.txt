cmake_minimum_required(VERSION 2.8.3)
project(dxl_armed_turtlebot)

find_package(catkin REQUIRED COMPONENTS roslaunch roseus rostest) # add roseus to gen messages

catkin_package()

add_rostest(test/test-dxl-armed-turtlebot.test)
roslaunch_add_file_check(launch/dxl_armed_turtlebot_bringup.launch)
roslaunch_add_file_check(launch/dxl_armed_turtlebot_gazebo.launch)
roslaunch_add_file_check(launch/hsi_color_filter.launch)
roslaunch_add_file_check(launch/turtlebot_joystick_teleop.launch)
