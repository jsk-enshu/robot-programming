cmake_minimum_required(VERSION 2.8.3)
project(turtleboteus)

find_package(catkin REQUIRED kobuki_msgs control_msgs move_base_msgs roseus rostest) # add roseus to gen messages

catkin_package()

if(CATKIN_ENABLE_TESTING)
  find_package(pr2eus QUIET)
  if(pr2eus_FOUND)
    add_rostest(test/test-turtlebot.test)
  endif()
endif()

