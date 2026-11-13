"""Jedy from scikit-robot: one interface for Gazebo, the real robot and Isaac Sim.

    from jedy_skrobot import connect

    robot, ri = connect()
    robot.reset_pose()
    ri.angle_vector(robot.angle_vector(), 3.0)
    ri.wait_interpolation()
    ri.go_velocity(0.1, 0.0, 0.0, duration=2.0)

Which robot answers is decided by what was launched, never by this code:

    ros2 launch jedy_bringup jedy_gazebo.launch.py            # Gazebo
    ros2 launch jedy_bringup jedy_fullbody_all.launch.py      # the real robot
    ros2 launch jedy_bringup jedy_isaac.launch.py             # Isaac Sim

All three publish the same topics under the same namespace (see
jedy_gazebo.launch.py), so the same script runs on any of them unchanged.

What this adds to actuo_ros's RobotInterface is what Jedy has beyond its
joints: the mecanum base (``go_velocity``), the IMU and the head camera. Each
of those raises a RuntimeError naming the missing topic when the running robot
does not have it, rather than returning a made-up zero. Servo ON/OFF works on
all three: the simulators serve the real robot's switch, and a released joint
falls under gravity there too.
"""

from jedy_skrobot.interface import BoardNotAnswering
from jedy_skrobot.interface import connect
from jedy_skrobot.interface import JedyInterface
from jedy_skrobot.interface import NAMESPACE

__all__ = ['BoardNotAnswering', 'JedyInterface', 'NAMESPACE', 'connect']
