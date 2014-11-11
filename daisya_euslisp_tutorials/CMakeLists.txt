cmake_minimum_required(VERSION 2.8.3)
project(daisya_euslisp_tutorials)

find_package(catkin REQUIRED COMPONENTS rostest)

catkin_package()

add_rostest(test/test-psolve.test)
add_rostest(test/test-maze.test)
add_rostest(test/test-daisya.test)
add_rostest(test/test-daisya-ik.test)
add_rostest(test/test-daisya-maze.test)
add_rostest(test/test-subsumption.test)
