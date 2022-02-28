#!/bin/bash

set -x
set -e

apt-get update
apt-get install -y sudo software-properties-common git wget sed sudo

# fix stopping tzdata for 18.04
echo 'debconf debconf/frontend select Noninteractive' | sudo debconf-set-selections

echo "Testing branch $TRAVIS_BRANCH of $REPOSITORY_NAME"
sudo sh -c "echo \"deb ${REPOSITORY} `lsb_release -cs` main\" > /etc/apt/sources.list.d/ros-latest.list"
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update -qq
sudo apt-get install -qq -y python-rosdep python-catkin-tools python-wstool ros-${ROS_DISTRO}-catkin
sudo rosdep init
rosdep update --include-eol-distros
# script:
(cd ${CI_SOURCE_PATH}; git log --oneline | head -10)
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
ln -sf ${CI_SOURCE_PATH} src/${REPOSITORY_NAME}
wstool init src
[ -e src/${REPOSITORY_NAME}/.rosinstall.${ROS_DISTRO} ] && wstool merge -t src src/${REPOSITORY_NAME}/.rosinstall.${ROS_DISTRO}
wstool update -t src
rosdep install --from-paths src -y -r -q --ignore-src --rosdistro ${ROS_DISTRO}
source /opt/ros/${ROS_DISTRO}/setup.bash
env | grep ROS
rosversion catkin
  # Build
catkin build -v -i --summarize --no-status -p 1 -j 1 --no-notify # every 5 mim
  # Run tests
catkin run_tests
  # check test (this only works on indigo)
catkin_test_results --verbose build

