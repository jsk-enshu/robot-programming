#!/bin/bash

trap 'exit 1' ERR
set -x

# 1. gazebo_ros_pkgs インストール
# http://gazebosim.org/wiki/Tutorials/1.9/Installing_gazebo_ros_Packages
# を参照

# 1-1. Remove ROS's Old Version of Gazebo ROS Integration
echo "1-1. Remove ROS's Old Version of Gazebo ROS Integration"
sudo apt-get remove ros-fuerte-simulator-gazebo ros-groovy-simulator-gazebo

# 1-2. Install from Source (on Ubuntu) 通りに以下を実行
echo "1-2. Install from Source (on Ubuntu) 通りに以下を実行"
source /opt/ros/groovy/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws
catkin_make

# 1-3. ここだけWebページとは違う
#echo "1-3. ここだけWebページとは違う"
#echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
cd ~/ros/groovy/
rosws merge ~/catkin_ws/devel/.rosinstall

# 1-4. git clone
echo "1-4. git clone"
source ~/.bashrc
cd ~/catkin_ws/src
git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git
git clone https://github.com/ros-controls/ros_control.git
git clone https://github.com/ros-controls/ros_controllers.git -b groovy-backported-hydro
git clone https://github.com/ros-controls/control_toolbox.git
git clone https://github.com/ros-controls/realtime_tools.git

# 1-5. 必要に応じて
echo "1-5. 必要に応じて"
cd ~/catkin_ws/src
rosdep update
rosdep check --from-paths . --ignore-src --rosdistro groovy
rosdep install --from-paths . --ignore-src --rosdistro groovy -y

# 1-6. catkin_makeを実行
echo "1-6. catkin_makeを実行"
source ~/.bashrc
cd ~/catkin_ws/
catkin_make
#もしpcl...cmakeがないというエラーが出た場合は、
#sudo aptitude install ros-groovy-pcl-conversions
#する

# gazebo_ros_pkgs 動作確認
#http://gazebosim.org/wiki/Tutorials/1.9/Using_roslaunch_Files_to_Spawn_Models
#roslaunch gazebo_ros willowgarage_world.launch


# 2. turtlebot実機インストール
echo "2. turtlebot実機インストール"
sudo apt-get install ros-groovy-turtlebot ros-groovy-turtlebot-apps ros-groovy-turtlebot-viz


# 3. turtlebot_simulator関係
# 3-1. kobukiのgazebo plugins
echo "3-1. kobukiのgazebo plugins"
git clone https://github.com/yujinrobot/kobuki_desktop.git ~/catkin_ws/src/kobuki_desktop
cd ~/catkin_ws
catkin_make

# 3-2. kobuki_description, turtlebot_description
echo "3-2. kobuki_description"
git clone --no-checkout https://github.com/yujinrobot/kobuki.git ~/catkin_ws/src/kobuki
cd ~/catkin_ws/src/kobuki
git reset HEAD kobuki.rosinstall README.markdown kobuki_description || echo "" # ignore error
git checkout kobuki.rosinstall README.markdown kobuki_description
echo "3-2. turtlebot_description"
git clone --no-checkout https://github.com/turtlebot/turtlebot.git ~/catkin_ws/src/turtlebot
cd ~/catkin_ws/src/turtlebot
git reset HEAD README.md setup_kobuki.sh turtlebot.rosinstall setup_create.sh turtlebot_description || echo "" # ignore error
git checkout README.md setup_kobuki.sh turtlebot.rosinstall setup_create.sh turtlebot_description

# 3-3. turtlebot_simulator
echo "3-3. turtlebot_simulator"
git clone https://github.com/turtlebot/turtlebot_simulator.git ~/catkin_ws/src/turtlebot_simulator
# patching
sed -i -e 's/bumper2pc.launch.xml/_bumper2pc.launch/g' ~/catkin_ws/src/turtlebot_simulator/turtlebot_gazebo/launch/includes/kobuki.launch.xml

# 3-4. for catkin_make of xx_description
echo "3-4. for catkin_make of xx_description"
git clone -b hydro-devel https://github.com/ros/xacro.git ~/catkin_ws/src/xacro
cd ~/catkin_ws/
catkin_make
