# jedyeus

EusLisp interface for jedy.

## Setup

### Prerequisites

### ros1_bridgeのUbuntu 24.04 (amd64) 用パッケージのインストール

以下のGoogle Driveリンクから .deb ファイルをダウンロードする．

- [ros-jazzy-ros1-bridge_0.10.3-0noble_amd64.deb](https://drive.google.com/file/d/1jXZlvovTGa_PU6stJOznvv5ZYkarB62n/view?usp=sharing)

ダウンロード後，以下のコマンドでインストールする．

```{code-block} console
$ sudo dpkg -i ros-jazzy-ros1-bridge_0.10.3-0noble_amd64.deb
```

依存関係のエラーが出た場合は以下のコマンドで解決する．

```{code-block} console
$ sudo apt-get install -f
```

```bash
# ROS-O
source /opt/ros/one/setup.bash
```

### jedyeusのセットアップ

```bash
# Create workspace
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src

# Clone this repository
git clone https://github.com/jsk-enshu/robot-programming.git

# Import dependencies using vcs (optional - imports opencv_apps and other packages)
cd ~/ros_ws
sudo apt install -y python3-vcstool
vcs import src < src/robot-programming/.repos.one

# Install dependencies using rosdep
source /opt/ros/one/setup.bash
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y -r

# Build
catkin build jedyeus
source ~/ros_ws/devel/setup.bash
```

## Usage

### Start Gazebo simulation in ROS 2

[jedyのGazebo環境設定](https://github.com/jsk-enshu/robot-programming/tree/master/jedy)に従ってGazebo環境をインストールして下記のコマンドからGazeboを起動する。

#### Terminal 1

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch jedy_bringup jedy_gazebo.launch.py
```

#### Terminal 2

```bash
source /opt/ros/one/setup.bash
roscore
```

#### Terminal 3

```bash
source /opt/ros/one/setup.bash
source /opt/ros/jazzy/setup.bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```

#### Terminal 4

```bash
source ~/ros_ws/devel/setup.bash
roscd jedyeus/euslisp
rosparam set /use_sim_time true
roseus jedy-interface.l
(jedy-init)
```
