cmake_minimum_required(VERSION 3.0.2)
project(mechatrobot)

## https://stackoverflow.com/questions/35856969/chrono-literals-is-not-a-namespace-name
if(CMAKE_COMPILER_IS_GNUCXX)
  execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
  if (GCC_VERSION VERSION_LESS 6.1)
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
  endif()
endif()

find_package(catkin REQUIRED COMPONENTS controller_manager cmake_modules diagnostic_updater hardware_interface std_msgs)
find_package(TinyXML REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  DEPENDS TinyXML)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${TinyXML_INCLUDE_DIRS})

## Declare a C++ executable for ROS_CONTROL
add_executable(robot_control src/robot_control.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(robot_control ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})

#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS robot_control
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch sketchbook urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslaunch roslint)
  file(GLOB LAUNCH_FILES launch/*.launch)
  foreach(LAUNCH_FILE ${LAUNCH_FILES})
    roslaunch_add_file_check(${LAUNCH_FILE})
  endforeach()
  
  file(GLOB CPP_FILES src/*.cpp)
  message("test ${CPP_FILES}")
  roslint_cpp(${CPP_FILES})
endif()
