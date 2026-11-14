#!/usr/bin/env python3
"""Move a hand with the keyboard: a target point, inverse kinematics, the robot.

The scikit-robot version of the 2025 EusLisp keyboard-ik-sample.l. Each key
moves the target (the red sphere in the viewer) by ``--step`` in the robot's
base frame; the arm is solved by IK to reach it and, if the IK succeeds, the
pose is sent to the robot. Keys are read from this terminal, not the viewer
window, so keep the terminal focused.

    ros2 launch jedy_bringup jedy_gazebo.launch.py     # or the real robot
    ros2 run jedy_skrobot keyboard_ik
    ros2 run jedy_skrobot keyboard_ik --model-only     # no robot: model only

Keys::

    f / b   forward / back      (x+ / x-)
    h / l   left / right        (y+ / y-)
    k / j   up / down           (z+ / z-)
    s       switch arm (right <-> left)
    e       look mode: the head looks at the target (e again: back to the arm)
    r       reset pose
    q       quit
"""

import argparse
import select
import sys
import termios
import tty

import numpy as np
from skrobot.coordinates import Coordinates
from skrobot.model.primitives import Sphere
from skrobot.viewers import PyrenderViewer

# key -> direction of the target's step, in the robot's base frame
# (x forward, y left, z up), as in keyboard-ik-sample.l.
MOVES = {
    'f': (1.0, 0.0, 0.0),
    'b': (-1.0, 0.0, 0.0),
    'h': (0.0, 1.0, 0.0),
    'l': (0.0, -1.0, 0.0),
    'k': (0.0, 0.0, 1.0),
    'j': (0.0, 0.0, -1.0),
}

HELP = """\
keyboard_ik: keys (type them in this terminal)
  f / b : forward / back      h / l : left / right      k / j : up / down
  s : switch arm (rarm <-> larm)
  e : look mode (the head looks at the target; e again: back to the arm)
  r : reset pose
  q : quit
"""


def read_key(timeout):
    """Return one character typed in the terminal, or None after timeout."""
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if not ready:
        return None
    return sys.stdin.read(1)


def hand_position(robot, arm):
    return getattr(robot, arm).end_coords.worldpos().copy()


def look_start(robot, distance):
    """A point ``distance`` [m] in front of the head camera."""
    camera = robot.head.end_coords.copy_worldcoords()
    camera.translate([0.0, 0.0, distance])   # optical frame: z is the view
    return camera.worldpos().copy()


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--step', type=float, default=0.02,
                   help='how far one key moves the target [m]')
    p.add_argument('--time', type=float, default=1.0,
                   help='seconds for each move sent to the robot')
    p.add_argument('--reset-time', type=float, default=3.0,
                   help='seconds to go to the reset pose')
    p.add_argument('--look-distance', type=float, default=0.3,
                   help='look mode starts this far in front of the camera [m]')
    p.add_argument('--model-only', action='store_true',
                   help='do not connect to a robot: move the model in the '
                        'viewer only')
    args = p.parse_args()

    if not sys.stdin.isatty():
        raise SystemExit('keyboard_ik reads keys from a terminal: run it in one')

    if args.model_only:
        from skrobot.models.jedy import Jedy
        robot, ri = Jedy(), None
    else:
        from jedy_skrobot import connect
        robot, ri = connect(load_meshes=True)
        ri.servo_on()                        # real robot: torque on, holding

    viewer = PyrenderViewer()
    viewer.add(robot)
    marker = Sphere(radius=0.01, color=[255, 0, 0])
    viewer.add(marker)
    viewer.show()

    def send(time):
        if ri is not None:
            ri.angle_vector(robot.angle_vector(), time)

    def reset():
        robot.reset_pose()                   # IK solves well from here
        send(args.reset_time)
        if ri is not None:
            ri.wait_interpolation()

    try:
        reset()
        arm = 'rarm'
        look = False
        target = hand_position(robot, arm)
        marker.newcoords(Coordinates(pos=target))
        viewer.redraw()

        print(HELP)
        print('{}  target {} m'.format(arm, np.round(target, 3)))

        old_tty = termios.tcgetattr(sys.stdin)
        try:
            # cbreak: one key at a time, no echo, but Ctrl-C still interrupts.
            tty.setcbreak(sys.stdin.fileno())
            while True:
                key = read_key(0.1)
                if key is None:
                    continue
                if key == 'q':
                    break
                if key == 'r':
                    reset()
                    target = (look_start(robot, args.look_distance) if look
                              else hand_position(robot, arm))
                    print('reset pose')
                elif key == 's':
                    look = False
                    arm = 'larm' if arm == 'rarm' else 'rarm'
                    target = hand_position(robot, arm)
                    print('switched to {}'.format(arm))
                elif key == 'e':
                    look = not look
                    target = (look_start(robot, args.look_distance) if look
                              else hand_position(robot, arm))
                    print('look mode: the head looks at the target' if look
                          else 'back to {}'.format(arm))
                elif key in MOVES:
                    goal = target + args.step * np.array(MOVES[key])
                    before = robot.angle_vector()
                    if look:
                        solved = robot.look_at(Coordinates(pos=goal))
                    else:
                        solved = getattr(robot, arm).inverse_kinematics(
                            Coordinates(pos=goal), rotation_axis=False,
                            warn_if_fail=False)
                    if solved is False or solved is None:
                        robot.angle_vector(before)
                        print('IK failed: target stays at {} m (try another key)'
                              .format(np.round(target, 3)))
                        continue
                    target = goal
                    send(args.time)
                else:
                    print(HELP)
                    continue
                print('{}  target {} m'.format(
                    'look' if look else arm, np.round(target, 3)))
                marker.newcoords(Coordinates(pos=target))
                viewer.redraw()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_tty)
    finally:
        if ri is not None:
            ri.stop()                        # the base too, also on Ctrl-C
    print('finished')


if __name__ == '__main__':
    main()
