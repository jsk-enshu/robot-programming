cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_7dof_arm)

find_package(catkin REQUIRED COMPONENTS rostest) # add roseus to gen messages

catkin_package()

if(CATKIN_ENABLE_TESTING)
  find_package(pr2eus QUIET)
  if(pr2eus_FOUND)
    add_rostest(test/test-dxl-7dof-arm.test)
  endif()
endif()

