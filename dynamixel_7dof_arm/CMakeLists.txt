cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_7dof_arm)

find_package(catkin REQUIRED COMPONENTS dynamixel_controllers rostest)

catkin_package()

add_rostest(test/test-dxl-7dof-arm.test)