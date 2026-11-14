#!/usr/bin/env python3
"""Send Nav2 goals from a program: fold the arms, then drive to each goal.

Gazebo only (docs/tips/slam.md): Nav2 needs the simulator's LiDAR.

    ros2 launch jedy_bringup jedy_gazebo.launch.py lidar:=true
    ros2 launch jedy_bringup jedy_nav2.launch.py rviz:=true
    ros2 run jedy_skrobot nav2_goal 3.0,0.4 3.0,0.4,90 0,0,180

Each goal is ``x,y[,yaw]`` in the map frame: metres, and degrees
counter-clockwise from the map's x axis (default 0). On the bundled map the
origin is the spawn pose, x ahead (along the room) and y to the left.

nav2_simple_commander's BasicNavigator does the talking to Nav2: goToPose()
sends a NavigateToPose goal, isTaskComplete() / getFeedback() follow it, and
getResult() says how it ended.
"""

import argparse
import math
import time

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from jedy_skrobot import connect


def make_pose(navigator, x, y, yaw_deg):
    """A map-frame goal pose.

    Parameters
    ----------
    navigator : BasicNavigator
        For the time stamp.
    x, y : float
        Position in the map [m].
    yaw_deg : float
        Heading [deg], counter-clockwise from the map's x axis.

    Returns
    -------
    geometry_msgs.msg.PoseStamped
    """
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    # A rotation about z only: the quaternion is (0, 0, sin(yaw/2), cos(yaw/2)).
    yaw = math.radians(yaw_deg)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def parse_goal(text):
    """``"x,y"`` or ``"x,y,yaw"`` -> ``(x, y, yaw_deg)``."""
    values = [float(v) for v in text.split(',')]
    if len(values) not in (2, 3):
        raise argparse.ArgumentTypeError(
            'a goal is x,y or x,y,yaw (got {!r})'.format(text))
    if len(values) == 2:
        values.append(0.0)
    return tuple(values)


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('goals', nargs='+', type=parse_goal,
                   help='x,y[,yaw]: map frame, metres and degrees')
    p.add_argument('--no-tuck', action='store_true',
                   help='leave the arms where they are')
    args = p.parse_args()

    robot, ri = connect()
    if not args.no_tuck:
        ri.tuck_arms()  # hanging, the hands drag on the floor

    navigator = BasicNavigator()
    # localizer='robot_localization' only means "do not set an initial pose":
    # with 'amcl' BasicNavigator publishes (0, 0, 0) as AMCL's initial pose,
    # which would throw away where AMCL already knows the robot is.
    # jedy_nav2.launch.py starts AMCL at (0, 0, 0) itself (nav2_params.yaml);
    # after moving the robot by hand, use RViz's "2D Pose Estimate".
    navigator.waitUntilNav2Active(localizer='robot_localization')

    for x, y, yaw in args.goals:
        start = time.monotonic()
        sim_start = ri.now()
        navigator.goToPose(make_pose(navigator, x, y, yaw))
        last_print = 0.0
        feedback = None
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback() or feedback
            if feedback is not None and time.monotonic() - last_print > 2.0:
                last_print = time.monotonic()
                print('  remaining {:.2f} m'.format(feedback.distance_remaining))
        result = navigator.getResult()
        here = feedback.current_pose.pose.position if feedback else None
        print('goal ({:.2f}, {:.2f}, {:.0f} deg): {}  {:.1f} s (robot clock {:.1f} s){}'.format(
            x, y, yaw, result.name, time.monotonic() - start, ri.now() - sim_start,
            '' if here is None else
            ', last pose ({:.2f}, {:.2f})'.format(here.x, here.y)))
        if result != TaskResult.SUCCEEDED:
            break

    navigator.destroy_node()


if __name__ == '__main__':
    main()
