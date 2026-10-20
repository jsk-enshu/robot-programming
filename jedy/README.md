# JEDY Robot (ROS2 Jazzy)

JEDY is a dual-arm mobile manipulator robot for ROS2 robot programming exercises.

詳細なマニュアルとインターフェースの説明は、以下のドキュメントをご覧ください：

- **オンラインマニュアル**: [http://jsk-enshu.github.io/robot-programming/](http://jsk-enshu.github.io/robot-programming/)

## ⚠️ **重要: ネットワーク設定**

**<span style="color:red">演習中や同一ネットワークで複数人が ROS2 を起動すると、通信が相互に干渉してしまいます。</span>**
**<span style="color:red">必ず以下の環境変数を設定してください:</span>**

```bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

**bashrc に追加して永続化する場合:**

```bash
echo 'export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST' >> ~/.bashrc
source ~/.bashrc
```

この設定により、ROS2 の通信がローカルホストのみに制限され、他の受講者との通信干渉を防ぐことができます。

> **注意:** ROS 2 Jazzy 以前では `ROS_LOCALHOST_ONLY=1` が使用されていましたが、Jazzy で非推奨となり削除されました。代わりに `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` を使用してください。詳細は [Improved Dynamic Discovery](https://docs.ros.org/en/jazzy/Tutorials/Advanced/Improved-Dynamic-Discovery.html) を参照してください。

## Features

- Differential drive base with 4 wheels
- Dual 7-DOF arms with grippers
- 2-DOF pan-tilt head with RGB-D camera
- 2D LiDAR sensor
- Full Gazebo Harmonic simulation support
- RViz2 visualization

## Packages

- **jedy_description**: URDF/xacro robot description files
- **jedy_bringup**: Launch files and configuration for simulation

## Setup

### Prerequisites

```bash
# ROS2 Jazzy
source /opt/ros/jazzy/setup.bash
```

### Build from source

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone this repository
git clone https://github.com/jsk-enshu/robot-programming.git

# Import dependencies using vcs (optional - imports opencv_apps and other packages)
cd ~/ros2_ws
sudo apt install -y python3-vcstool
vcs import src < src/robot-programming/.repos.jazzy

# Install dependencies using rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y -r

# Build
colcon build --symlink-install
source install/setup.bash
```

## Usage

### Start Gazebo simulation with RViz

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch jedy_bringup jedy_gazebo.launch.py
```

This will start:
- Gazebo Harmonic simulator
- RViz2 visualization
- All robot controllers (mecanum_drive, head, arms)
- Camera and LiDAR bridges
- Point cloud generation from RGB-D

### Control the robot

#### Move the base (mecanum drive)

```bash
# Using ros2 topic pub
ros2 topic pub /mecanum_drive_controller/reference geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}" \
  --rate 10

# Or using teleop_twist_keyboard (optional, not included in rosdep)
# Install with: sudo apt install ros-jazzy-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/mecanum_drive_controller/reference -p stamped:=true
```

#### Control the arms

```bash
# List available controllers
ros2 control list_controllers

# Example: Move right arm
ros2 topic pub /rarm_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['rarm_joint0', 'rarm_joint1', 'rarm_joint2', 'rarm_joint3', 'rarm_joint4', 'rarm_joint5', 'rarm_joint6'],
    points: [{positions: [0.0, 0.0, 0.0, -1.57, 0.0, 0.0, 0.0], time_from_start: {sec: 2}}]}" \
  --once
```

#### Control the head

```bash
# Pan and tilt the head
ros2 topic pub /head_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['head_joint0', 'head_joint1'],
    points: [{positions: [0.5, 0.3], time_from_start: {sec: 1}}]}" \
  --once
```

#### Run the sample scripts in simulation

The sample scripts in `jedy_bringup/scripts` set `header.stamp` from the node clock.
In Gazebo the controllers run on simulation time, so pass `use_sim_time:=true`:

```bash
ros2 run jedy_bringup send_joint_position.py --ros-args -p use_sim_time:=true
ros2 run jedy_bringup track_box_to_cmd_vel_1.py --ros-args -p use_sim_time:=true
```

Without it, `send_joint_position.py` never starts the head trajectory, `track_box_to_cmd_vel_0.py` leaves the base moving after the commands stop (`reference_timeout` is not applied), and `track_box_to_cmd_vel_1.py` treats every detection as stale and does not move the base.

### Available topics

```bash
# Check all topics
ros2 topic list

# Important topics:
# /mecanum_drive_controller/reference    - Base velocity commands (TwistStamped)
# /camera/rgb/image_raw                  - RGB camera image
# /camera/depth/image_raw                - Depth image
# /camera/depth/points                   - Point cloud (XYZRGB)
# /scan                                  - 2D LiDAR scan
# /joint_states                          - All joint states
# /mecanum_drive_controller/odometry     - Wheel odometry from mecanum drive
# /odom                                  - Odometry fused by EKF (wheel odometry + IMU)
# /rarm_controller/joint_trajectory      - Right arm control
# /larm_controller/joint_trajectory      - Left arm control
# /head_controller/joint_trajectory      - Head control
```

### Check controller status

```bash
# List controllers
ros2 control list_controllers

# Get controller info
ros2 control list_hardware_interfaces
```

## Configuration

### Controllers

Controller configuration is in `jedy_bringup/config/jedy_controllers.ros2.yaml`:

- **mecanum_drive_controller**: Controls 4 mecanum wheels
  - Wheels: `front_left_wheel_joint`, `front_right_wheel_joint`, `rear_left_wheel_joint`, `rear_right_wheel_joint`
  - Wheel radius: 0.030083 m
  - Velocity command: `/mecanum_drive_controller/reference` (TwistStamped, `reference_timeout: 0.5` s)

- **head_controller**: Joint trajectory controller for 2-DOF head
- **rarm_controller**: Joint trajectory controller for 7-DOF right arm
- **larm_controller**: Joint trajectory controller for 7-DOF left arm

### Initial joint positions

Modify initial joint positions in `jedy_bringup/launch/jedy_gazebo.launch.py`:

```python
robot_description = {'robot_description': Command([
    'xacro ', model_file,
    ' head_j0_init:=0.0',              # 0 deg
    ' head_j1_init:=0.0',              # 0 deg
    ' rarm_j0_init:=1.5708',           # 90 deg
    ' rarm_j1_init:=0.0',              # 0 deg
    # ... modify as needed
])}
```

## RViz Configuration

The default RViz configuration is in `jedy_bringup/config/jedy.rviz`.

To save your own configuration:
1. Open RViz manually: `rviz2`
2. Load topics and adjust views
3. File -> Save Config As -> `jedy_bringup/config/jedy.rviz`

## Troubleshooting

### Controllers fail to load

Wait for gz_ros2_control plugin to initialize. Controllers are spawned with delays:
- joint_state_broadcaster: 5s
- mecanum_drive_controller: 7s
- head_controller: 9s
- rarm_controller: 11s
- larm_controller: 13s

### Robot falls through ground

Check that Gazebo has fully loaded before spawning the robot. Increase spawn delay if needed.

### Camera/LiDAR topics not available

Ensure ros_gz_bridge nodes are running:
```bash
ros2 node list | grep bridge
```

### TF errors

Check that all required TF frames are published:
```bash
ros2 run tf2_tools view_frames
```

## Architecture

- **Simulation**: Gazebo Harmonic
- **Control**: ros2_control with gz_ros2_control plugin
- **Base**: mecanum_drive_controller
- **Arm Control**: joint_trajectory_controller
- **Sensors**: RGB-D camera via ros_gz_bridge, 2D LiDAR via ros_gz_bridge

## Related Links

- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic)
- [ros2_control](https://control.ros.org/jazzy/index.html)
