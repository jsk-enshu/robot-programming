#!/bin/bash

set -x

apt-get update
apt-get install -y sudo software-properties-common git wget sed

echo "Testing branch $TRAVIS_BRANCH of $REPOSITORY_NAME"
sudo sh -c "echo \"deb ${REPOSITORY} `lsb_release -cs` main\" > /etc/apt/sources.list.d/ros-latest.list"
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update -qq
sudo apt-get install -qq -y python-rosdep python-catkin-tools python-wstool ros-${ROS_DISTRO}-catkin
  # https://github.com/ros/ros_comm/pull/668
sudo apt-get install -qq -y ros-${ROS_DISTRO}-rostest
(cd /opt/ros/$ROS_DISTRO/lib/python2.7/dist-packages; wget --no-check-certificate https://patch-diff.githubusercontent.com/raw/ros/ros_comm/pull/637.diff -O - | sudo patch -f -p4 || echo "ok" )
(cd /opt/ros/$ROS_DISTRO/lib/python2.7/dist-packages; wget --no-check-certificate https://patch-diff.githubusercontent.com/raw/ros/ros_comm/pull/668.diff -O - | sudo patch -f -p4 || echo "ok" )
sudo rosdep init
rosdep update
# script:
(cd ${CI_SOURCE_PATH}; git log --oneline | head -10)
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
ln -sf ${CI_SOURCE_PATH} src/${REPOSITORY_NAME}
wstool init src
wstool set dynamixel_urdf https://github.com/jsk-enshu/dynamixel_urdf --git -t src -y
wstool update -t src
rosdep install --from-paths src -y --ignore-src --rosdistro ${ROS_DISTRO}
source /opt/ros/${ROS_DISTRO}/setup.bash
env | grep ROS
rosversion catkin
  # Build
catkin build -v -i --summarize --no-status -p 1 -j 1 --no-notify # every 5 mim
  # Run tests
catkin run_tests
  # check test (this only works on indigo)
catkin_test_results --verbose build

