# JEDY Robot (ROS2 Jazzy)

JEDY is a dual-arm mobile manipulator robot for ROS2 robot programming exercises.

## ⚠️ **重要: ネットワーク設定**

**<span style="color:red">演習中や同一ネットワークで複数人が ROS2 を起動すると、通信が相互に干渉してしまいます。</span>**
**<span style="color:red">必ず以下の環境変数を設定してください:</span>**

```bash
export ROS_LOCALHOST_ONLY=1
```

**bashrc に追加して永続化する場合:**

```bash
echo 'export ROS_LOCALHOST_ONLY=1' >> ~/.bashrc
source ~/.bashrc
```

この設定により、ROS2 の通信がローカルホストのみに制限され、他の受講者との通信干渉を防ぐことができます。

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
git clone -b ros2 https://github.com/iory/robot-programming.git

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
- All robot controllers (diff_drive, head, arms)
- Camera and LiDAR bridges
- Point cloud generation from RGB-D

### Control the robot

#### Move the base (differential drive)

```bash
# Using ros2 topic pub
ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}" \
  --rate 10

# Or using teleop_twist_keyboard (optional, not included in rosdep)
# Install with: sudo apt install ros-jazzy-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel
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

### Available topics

```bash
# Check all topics
ros2 topic list

# Important topics:
# /diff_drive_controller/cmd_vel         - Base velocity commands (TwistStamped)
# /camera/rgb/image_raw                  - RGB camera image
# /camera/depth/image_raw                - Depth image
# /camera/depth/points                   - Point cloud (XYZRGB)
# /scan                                  - 2D LiDAR scan
# /joint_states                          - All joint states
# /odom                                  - Odometry from differential drive
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

- **diff_drive_controller**: Controls 4 wheels for differential drive
  - Left wheels: `front_left_wheel_joint`, `rear_left_wheel_joint`
  - Right wheels: `front_right_wheel_joint`, `rear_right_wheel_joint`
  - Wheel separation: 0.101 m
  - Wheel radius: 0.030083 m

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
- diff_drive_controller: 7s
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
- **Differential Drive**: diff_drive_controller (supports multiple wheels per side)
- **Arm Control**: joint_trajectory_controller
- **Sensors**: RGB-D camera via ros_gz_bridge, 2D LiDAR via ros_gz_bridge

## Related Links

- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic)
- [ros2_control](https://control.ros.org/jazzy/index.html)
