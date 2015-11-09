cmake_minimum_required(VERSION 2.8.3)
project(dxl_armed_turtlebot)

find_package(catkin REQUIRED COMPONENTS roseus rostest) # add roseus to gen messages

catkin_package()

_generate_eus_dep_msgs(${PROJECT_NAME})

add_rostest(test/test-dxl-armed-turtlebot.test)
