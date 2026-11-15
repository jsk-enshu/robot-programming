"""Jedy from scikit-robot: one interface for Gazebo and the real robot.

    from jedy_skrobot import connect

    robot, ri = connect()
    robot.reset_pose()
    ri.angle_vector(robot.angle_vector(), 3.0)
    ri.wait_interpolation()
    ri.go_velocity(0.1, 0.0, 0.0, duration=2.0)

Which robot answers is decided by what was launched, never by this code:

    ros2 launch jedy_bringup jedy_gazebo.launch.py            # Gazebo
    ros2 launch jedy_bringup jedy_fullbody_all.launch.py      # the real robot

Both publish the same topics under the same namespace (see
jedy_gazebo.launch.py), so the same script runs on either unchanged.

What this adds to actuo_ros's RobotInterface is what Jedy has beyond its
joints: the mecanum base (``go_velocity``), the IMU and the head camera. Each
of those raises a RuntimeError naming the missing topic when the running robot
does not have it, rather than returning a made-up zero. Servo ON/OFF works on
all three: the simulators serve the real robot's switch, and a released joint
falls under gravity there too.
"""

try:
    from jedy_skrobot.interface import BoardNotAnswering
    from jedy_skrobot.interface import connect
    from jedy_skrobot.interface import JedyInterface
    from jedy_skrobot.interface import NAMESPACE
    from jedy_skrobot.interface import VlmRefused
except ModuleNotFoundError as e:
    # scikit-robot and the other dependencies live in the package's own venv,
    # not in the system python3 (a mock student ran `python3 my.py` and the
    # bare "No module named 'skrobot'" did not say what to do).
    if e.name is None or e.name.split('.')[0] in ('rclpy', 'jedy_skrobot'):
        raise
    raise ModuleNotFoundError(
        "{}. jedy_skrobot runs on its own venv's python, not python3: "
        "$(ros2 pkg prefix jedy_skrobot)/share/jedy_skrobot/venv/bin/python "
        "<your program>.py (see the day-2 docs, プログラムの置き場所と実行)".format(e),
        name=e.name) from e

__all__ = ['BoardNotAnswering', 'JedyInterface', 'NAMESPACE', 'VlmRefused', 'connect']
