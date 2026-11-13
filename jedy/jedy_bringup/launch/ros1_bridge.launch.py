import os
import subprocess
import sys

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    # Check if ros1_bridge package exists
    try:
        result = subprocess.run(
            ['ros2', 'pkg', 'list'],
            capture_output=True,
            text=True,
            check=True
        )
        if 'ros1_bridge' not in result.stdout.split('\n'):
            raise RuntimeError("ros1_bridge package not found")
    except (subprocess.CalledProcessError, RuntimeError):
        print("\n" + "="*70, file=sys.stderr)
        print("ERROR: ros1_bridge package not found!", file=sys.stderr)
        print("="*70, file=sys.stderr)
        print("\nPlease install ros1_bridge using the following steps:\n", file=sys.stderr)
        print("For Ubuntu 24.04 (amd64):", file=sys.stderr)
        print("1. Download the .deb file from Google Drive:", file=sys.stderr)
        print("   ros-jazzy-ros1-bridge_0.10.3-0noble_amd64.deb\n", file=sys.stderr)
        print("2. Install the package:", file=sys.stderr)
        print("   sudo dpkg -i ros-jazzy-ros1-bridge_0.10.3-0noble_amd64.deb\n", file=sys.stderr)
        print("3. If you encounter dependency errors, fix them with:", file=sys.stderr)
        print("   sudo apt-get install -f\n", file=sys.stderr)
        print("="*70 + "\n", file=sys.stderr)
        sys.exit(1)

    # Build source command based on existing environments
    source_commands = []

    ros1_setup = '/opt/ros/one/setup.bash'
    if os.path.exists(ros1_setup):
        source_commands.append(f'source {ros1_setup}')

    ros2_setup = '/opt/ros/jazzy/setup.bash'
    if os.path.exists(ros2_setup):
        source_commands.append(f'source {ros2_setup}')

    # Get config file path
    config_path = os.path.join(os.path.dirname(__file__), '..', 'config', 'controller_bridge.yaml')
    config_path = os.path.abspath(config_path)

    # Get robot_description publisher script path
    script_path = os.path.join(os.path.dirname(__file__), '..', 'scripts', 'publish_robot_description.py')
    script_path = os.path.abspath(script_path)

    # Combine source commands with bridge execution
    bridge_cmd = f'ros2 run ros1_bridge dynamic_bridge --bridge-all-topics --config {config_path}'
    full_cmd = ' && '.join(source_commands + [bridge_cmd])

    ros1_bridge = ExecuteProcess(
        cmd=['bash', '-c', full_cmd],
        shell=False,
        output='screen'
    )

    # Launch robot_description publisher
    robot_description_publisher = ExecuteProcess(
        cmd=['python3', script_path],
        output='screen'
    )

    return LaunchDescription([
        robot_description_publisher,
        ros1_bridge,
    ])
