if(NOT USE_ROSBUILD)
  include(catkin.cmake)
  return()
endif()
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

# rosbuild_find_ros_package(dynamixel_7dof_arm)
# rosbuild_find_ros_package(turtlebot_description)
# add_custom_command(
#   OUTPUT ${PROJECT_SOURCE_DIR}/models/kobuki_hexagons_kinect.urdf
#   COMMAND rosrun xacro xacro.py ${turtlebot_description_PACKAGE_PATH}/robots/kobuki_hexagons_kinect.urdf.xacro > ${PROJECT_SOURCE_DIR}/models/kobuki_hexagons_kinect.urdf
#   DEPENDS ${turtlebot_description_PACKAGE_PATH}/robots/kobuki_hexagons_kinect.urdf.xacro
#   )
# add_custom_command(
#   OUTPUT ${PROJECT_SOURCE_DIR}/models/kobuki_hexagons_kinect.dae
#   COMMAND rosrun collada_urdf urdf_to_collada ${PROJECT_SOURCE_DIR}/models/kobuki_hexagons_kinect.urdf ${PROJECT_SOURCE_DIR}/models/kobuki_hexagons_kinect.dae
#   DEPENDS ${PROJECT_SOURCE_DIR}/models/kobuki_hexagons_kinect.urdf
#   )
# add_custom_command(
#   OUTPUT ${PROJECT_SOURCE_DIR}/models/kobuki_hexagons_kinect.l
#   COMMAND rosrun euscollada collada2eus ${PROJECT_SOURCE_DIR}/models/kobuki_hexagons_kinect.dae ${PROJECT_SOURCE_DIR}/models/kobuki_hexagons_kinect.l
#   DEPENDS ${PROJECT_SOURCE_DIR}/models/kobuki_hexagons_kinect.dae
#   )
# add_custom_target(compile_robots ALL DEPENDS ${PROJECT_SOURCE_DIR}/models/kobuki_hexagons_kinect.l)
