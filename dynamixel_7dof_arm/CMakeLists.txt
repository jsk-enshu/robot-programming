cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_7dof_arm)

find_package(catkin REQUIRED COMPONENTS dynamixel_controllers roseus rostest) # add roseus to gen messages

catkin_package()

add_rostest(test/test-dxl-7dof-arm.test)