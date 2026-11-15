#!/usr/bin/env python3
"""Pick up an object on the floor in front of the robot (object detection + depth).

    ros2 launch jedy_bringup jedy_fullbody_all.launch.py camera:=true   # or Gazebo
    ros2 launch jedy_vision recognition.launch.py
    ros2 run jedy_vision grasp_object "rubik's cube"
    ros2 run jedy_vision grasp_object "rubik's cube" --dry-run   # look and plan only

1. look: the object detector (``ri.what_do_you_see``) finds the object, and
   the depth pixels in its box give an upright box around it on the floor
   (``jedy_vision.object_box``: centre, size and yaw). One depth reading
   would not do: seen from above it lands on the top face, and taking it
   for the front put the centre 1-1.5 cm too far (real robot).
2. plan: the hand comes down onto the object from in front, pitched down.
   A level hand (as grasp_tag holds it) does not reach the floor: with the
   model, no position 0.14-0.26 m ahead solved. Pitched down it does, the
   nearer the steeper. The pitches are tried from straight down (90 deg)
   up to 15 deg, and the steepest for which the whole motion solves is used
   -- a shallow hand brings its lower finger near the floor (the fingers
   reach ~1.9 cm below the hand's centre line, grasp_tag; a 4.6 cm object's
   centre is 2.3 cm up). The motion: to above the point in front of the
   object, straight down to it, straight in, and straight up. With the
   model, right arm, 0.084 m right, from ARMS_CLEAR: it solved 0.12-0.14 m
   ahead (75 deg) and 0.16-0.18 m (55-20 deg), not at 0.15 or 0.19-0.21 m.
3. if no pitch reaches it, the base drives straight forward or back to
   bring the object to REACH_X ahead, and it looks and plans again.
4. grasp: open, go in, close, lift; held if the fingers stopped on
   something instead of closing on nothing. Then put it back down where it
   was, let go and go back.
"""

import argparse
import math

import numpy as np

from jedy_skrobot import connect
from jedy_vision import ARMS_CLEAR, object_box, set_head, solve_ik

#: Pitches of the approach below level [deg], tried in this order.
PITCHES = tuple(range(90, 14, -5))
#: Where the object should be ahead of base_link [m] when the base has to
#: move: the middle of the widest band a steep hand reaches (0.12-0.14 m,
#: see the module docstring).
REACH_X = 0.13
#: Base pushes: no slower than the real base moves (~0.078 m/s), no shorter
#: than this [m] (shorter ones do not move it).
BASE_SPEED = 0.08
MIN_PUSH = 0.03
#: [m] The hand stops this far back along the approach before moving in,
#: and comes down onto that point from this far above it.
PRE = 0.05
ABOVE = 0.05
#: Straight lines are solved at points this far apart [m]; a step needing a
#: joint change over MAX_JUMP [deg] is refused (the arm would swing).
STEP = 0.01
MAX_JUMP = 12.0
#: Arm joint speed limit [deg/s] (grasp_tag: faster swings turn the robot).
ARM_SPEED = 30.0
#: Gripper [rad]: open, and the closing command (past the object; the
#: fingers stop on it). Positive opens. Closed on nothing, the real
#: grippers stop at about -0.39 rad (jedy_fullbody_rcb4.yaml).
GRIPPER_OPEN, GRIPPER_CLOSE = 0.8, -0.3
#: Stopped above this [rad] after closing, the fingers are on something.
HOLDING_ABOVE = -0.25
#: [deg] After each arm motion, a joint this far from its command stops the
#: program: the arm is blocked (by the robot's own base, the floor, the
#: object) or a servo is not following. On the real robot the inverse
#: kinematics, which knows nothing of the robot's own body, folded the right
#: forearm against the base for an object 0.14 m ahead; the shoulder
#: (rarm_joint1) then stayed at -92 deg whatever it was sent, 38-123 deg
#: from its command, and the hand ended 8-10 cm from the target.
STALL_DEG = 15.0


def find(ri, robot, word, tries=5):
    """The object's box (``jedy_vision.ObjectBox``, base frame), or None."""
    K, _ = ri.camera_info()
    for _ in range(tries):
        try:
            found = ri.what_do_you_see([word])
        except Exception as e:   # recognition.launch.py not running, ...
            raise SystemExit('object detection failed: {}'.format(e))
        if not found:
            continue
        d = max(found, key=lambda d: d.score)
        depth, _ = ri.depth_image()
        robot.angle_vector(ri.angle_vector())
        box = object_box(depth, d.box, K, robot)
        if box is None:
            print('{} ({:.2f}): too few depth points in its box'.format(d.label, d.score))
            continue
        print('{} ({:.2f}): centre {} m, {} cm, yaw {:+.0f} deg ({})'.format(
            d.label, d.score, np.round(box.position, 3), np.round(100 * box.size, 1),
            math.degrees(box.yaw), box.method))
        return box
    return None


def straight(robot, arm, start, goal, rotation):
    """Angle vectors moving the hand in a straight line (None if it fails)."""
    from skrobot.coordinates import Coordinates
    n = max(1, int(math.ceil(np.linalg.norm(goal - start) / STEP)))
    path = []
    for k in range(1, n + 1):
        before = np.array([j.joint_angle() for j in arm.joint_list])
        ok = arm.inverse_kinematics(
            Coordinates(pos=start + (goal - start) * k / float(n), rot=rotation),
            rotation_axis=True, warn_if_fail=False)
        if ok is False or ok is None:
            return None
        after = np.array([j.joint_angle() for j in arm.joint_list])
        if np.degrees(np.max(np.abs(after - before))) > MAX_JUMP:
            return None
        path.append(robot.angle_vector().copy())
    return path


def plan(robot, arm, box, lift, start):
    """The arm motions to grasp ``box`` and lift; None if out of reach.

    The hand comes in along the box's x axis (the one nearest the robot's
    +x) and the fingers close along its y: across two opposite faces.

    As grasp_tag: the most constrained pose, the grasp, is solved first (from
    several start poses), the pose PRE before it from that solution, and the
    rest are straight lines traced from there: up from 'pre' (lower if
    ABOVE does not trace), in to the grasp, and up to the lift. Solving
    'above' first instead failed for most positions (model).
    """
    for pitch in PITCHES:
        t = math.radians(pitch)
        c, s = math.cos(box.yaw), math.sin(box.yaw)
        approach = np.array([math.cos(t) * c, math.cos(t) * s, -math.sin(t)])   # fingers point
        closing = np.array([-s, c, 0.0])                         # the axis they close along
        R = np.column_stack([approach, closing, np.cross(approach, closing)])
        grasp = box.position
        pre = grasp - PRE * approach
        up = grasp + np.array([0.0, 0.0, lift])
        for rot in (R, R.dot(np.diag([1.0, -1.0, -1.0]))):   # fingers swapped
            robot.angle_vector(start)
            if not solve_ik(robot, arm, grasp, rotation=rot):
                continue
            if not solve_ik(robot, arm, pre, rotation=rot):
                continue
            at_pre = robot.angle_vector().copy()
            down = None
            for height in (ABOVE, 0.6 * ABOVE, 0.3 * ABOVE):
                robot.angle_vector(at_pre)
                top = pre + np.array([0.0, 0.0, height])
                traced = straight(robot, arm, pre, top, rot)
                if traced is not None:
                    above = traced[-1]
                    down = traced[::-1][1:] + [at_pre]
                    break
            if down is None:
                continue
            robot.angle_vector(at_pre)
            reach_in = straight(robot, arm, pre, grasp, rot)
            if reach_in is None:
                continue
            raise_up = straight(robot, arm, grasp, up, rot)
            if raise_up is None:
                continue
            robot.angle_vector(start)
            return {'pitch': pitch, 'above': above, 'down': down, 'in': reach_in,
                    'up': raise_up, 'targets': {'above': top, 'pre': pre,
                                                'grasp': grasp, 'lift': up}}
    robot.angle_vector(start)
    return None


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('word', nargs='?', default="rubik's cube",
                   help='what to pick up, as ri.what_do_you_see takes it')
    p.add_argument('--tilt', type=float, default=50.0, help='head tilt, down [deg]')
    p.add_argument('--lift', type=float, default=0.05, help='[m]')
    p.add_argument('--time', type=float, default=3.0, help='seconds per arm motion')
    p.add_argument('--open', type=float, default=GRIPPER_OPEN, help='gripper open [rad]')
    p.add_argument('--close', type=float, default=GRIPPER_CLOSE,
                   help='gripper closing command, past the object [rad]')
    p.add_argument('--dry-run', action='store_true',
                   help='look and plan only: only the head moves')
    args = p.parse_args()

    robot, ri = connect()
    try:
        grasp(args, robot, ri)
    finally:
        ri.stop()   # the base, also on Ctrl-C while it drives


def grasp(args, robot, ri):
    if args.dry_run:
        ri.servo_on(joints=['head_joint0', 'head_joint1'])
    else:
        ri.servo_on()
        # Hands high and out first: from wherever the arms are, a direct
        # move to the object could sweep across it.
        ri.tuck_arms(time=5.0, pose=ARMS_CLEAR)
    set_head(robot, ri, 0.0, args.tilt)

    for attempt in range(4):
        box = find(ri, robot, args.word)
        if box is None:
            raise SystemExit('no "{}" in view (head tilted {:.0f} deg down)'.format(
                args.word, args.tilt))
        centre = box.position
        side = 'larm' if centre[1] > 0.02 else 'rarm'
        arm = getattr(robot, side)
        robot.angle_vector(ri.angle_vector())
        home = robot.angle_vector().copy()
        poses = plan(robot, arm, box, args.lift, home)
        if poses is not None:
            break
        push = centre[0] - REACH_X
        print('{} cannot reach it {:.3f} m ahead with any pitch'.format(side, centre[0]))
        if args.dry_run:
            raise SystemExit('dry run: the base would drive {:+.3f} m'.format(push))
        if attempt == 3:
            raise SystemExit('could not bring it into reach')
        if abs(push) < MIN_PUSH:
            push = math.copysign(MIN_PUSH, push)
        print('driving {:+.3f} m'.format(push))
        ri.go_velocity(math.copysign(BASE_SPEED, push), 0.0, 0.0,
                       duration=abs(push) / BASE_SPEED)
        ri.stop()
        ri.sleep(1.0)   # let the base settle before looking
    print('{} grasps it pitched {} deg down'.format(side, poses['pitch']))
    if box.size[1] > 0.065:
        print('note: {:.1f} cm across is about as wide as the fingers open'.format(
            100 * box.size[1]))
    if args.dry_run:
        print('dry run: not moving the arm')
        return

    gripper = getattr(robot, '{}_gripper_joint'.format(side))
    arm_ctrl = '{}_arm_controller'.format('left' if side == 'larm' else 'right')
    grip_ctrl = '{}_gripper_controller'.format('left' if side == 'larm' else 'right')

    def move_arm(avs, seconds, name=None):
        """Through ``avs`` (one angle vector or a list), no faster than ARM_SPEED."""
        avs = [avs] if isinstance(avs, np.ndarray) else list(avs)
        robot.angle_vector(ri.angle_vector())
        prev = np.array([j.joint_angle() for j in arm.joint_list])
        times = []
        for av in avs:
            robot.angle_vector(av)
            now = np.array([j.joint_angle() for j in arm.joint_list])
            times.append(max(seconds / len(avs),
                             np.degrees(np.max(np.abs(now - prev))) / ARM_SPEED))
            prev = now
        if len(avs) == 1:
            ri.angle_vector(avs[0], times[0], controller_type=arm_ctrl)
        else:
            ri.angle_vector_sequence(avs, times, controller_type=arm_ctrl)
        robot.angle_vector(avs[-1])
        commanded = {j.name: j.joint_angle() for j in arm.joint_list}
        ri.wait_interpolation(arm_ctrl)
        ri.sleep(0.3)
        robot.angle_vector(ri.angle_vector())
        worst = max(arm.joint_list, key=lambda j: abs(j.joint_angle() - commanded[j.name]))
        off = math.degrees(worst.joint_angle() - commanded[worst.name])
        if name is not None:
            err = np.linalg.norm(arm.end_coords.worldpos() - poses['targets'][name])
            print('{}: hand {:.1f} mm from the target, worst joint {} {:+.1f} deg'.format(
                name, 1000 * err, worst.name, off))
        if abs(off) > STALL_DEG:
            raise SystemExit('STOP: {} is {:+.1f} deg from its command -- the arm is '
                             'blocked or the servo is not following. Check the robot '
                             'before moving it again.'.format(worst.name, off))

    def move_gripper(angle, seconds=1.0):
        robot.angle_vector(ri.angle_vector())
        gripper.joint_angle(angle)
        ri.angle_vector(robot.angle_vector(), seconds, controller_type=grip_ctrl)
        ri.wait_interpolation(grip_ctrl)
        robot.angle_vector(ri.angle_vector())
        return gripper.joint_angle()

    move_arm(poses['above'], args.time, 'above')
    move_gripper(args.open)
    move_arm(poses['down'], 1.5, 'pre')
    move_arm(poses['in'], 2.0, 'grasp')
    move_gripper(args.close, 1.5)
    move_arm(poses['up'], 2.0, 'lift')
    ri.sleep(0.5)
    robot.angle_vector(ri.angle_vector())
    angle = gripper.joint_angle()
    held = angle > HOLDING_ABOVE
    print('{}: gripper stopped at {:+.2f} rad (closed on nothing: about -0.39)'.format(
        'HELD' if held else 'NOT HELD', angle))
    ri.sleep(1.0)
    # Put it back down where it was, let go, and go back the way it came.
    move_arm(poses['up'][::-1][1:] + [poses['in'][-1]], 2.0, 'grasp')
    move_gripper(args.open)
    move_arm(poses['in'][::-1][1:] + [poses['down'][-1]], 2.0, 'pre')
    move_arm(poses['down'][::-1][1:] + [poses['above']], 1.5, 'above')
    move_gripper(0.0)
    move_arm(home, args.time)
    if not held:
        raise SystemExit(1)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('stopped by Ctrl-C')
        raise SystemExit(130)
