#!/usr/bin/env python3
"""Pick up the tag cube with one hand (inverse kinematics with orientation).

    ros2 launch jedy_bringup jedy_gazebo.launch.py demo:=true camera:=d405
    ros2 run jedy_vision approach_tag --standoff 0.17   # stand in front of the stand
    ros2 run jedy_vision grasp_tag

The cube has a different AprilTag on each of its six faces, so whichever
faces are in view, each one gives the cube's centre and orientation
(``cube_pose``). From that pose:

1. the hand is turned so its fingers close across two opposite side faces,
   approaching along the side axis that points most nearly away from the
   robot (a 7-joint arm: position AND orientation, 6 constraints);
2. it stops 5 cm short, opens, moves straight in, and closes;
3. it lifts the cube, and checks that it did -- the gripper stopped on the
   cube instead of closing on nothing, and, if the camera can see the cube
   past the hand (it usually cannot), that the cube is higher;
4. it puts the cube back where it was picked up, lets go, backs off and
   looks again to check that the cube is there.

Closing, the fingers squeeze the cube out towards their tips (see
HELD_AT). Put down at the grasp pose, it ended up 1.5-2.5 cm further away
every run (Gazebo), and fell off the back of the stand on the second or
third. So the hand puts it down that much nearer.

If the arm cannot do all that from where the robot stands, the base drives
straight towards or away from the cube (forward/backward only) until it can.
"""

import argparse
import math

import numpy as np

from jedy_skrobot import connect
from jedy_vision import (ARMS_CLEAR, TAG_CUBE_EDGE, TAG_CUBE_TAG, TagDetector,
                         cube_pose, say, set_head, solve_ik)

#: Where the cube's centre should be ahead of base_link for the hand to reach
#: it, slide in and lift with the hand level [m]. Narrow: IK over a grid at
#: the cube's height on the demo stand (base_link's height) solved the whole
#: motion for every y from 0 to -0.08 m (right arm) only at x = 0.20 m; at
#: 0.19 and 0.21 m some y failed, at 0.22 m most did.
REACH_X = (0.194, 0.206)
#: Gripper joint [rad]: open (fingertips ~6-7 cm apart) and the closing
#: command (past the cube: the fingers stop on it, limited by the servo's
#: torque). Signs as in jedy_description's jedy_gz.xacro, which Gazebo
#: simulates and the real robot's robot_state_publisher loads: positive
#: opens (checked in Gazebo, both fingers). scikit-robot's own Jedy model
#: has this joint's axis the other way round; whether the real servo turns
#: as jedy_gz.xacro says is NOT checked here -- see --open/--close.
GRIPPER_OPEN, GRIPPER_CLOSE = 0.5, -0.3
#: end_coords (between the fingertips) sits this far beyond the cube's centre
#: along the approach: the finger pads (8-10 cm from the wrist) are then on
#: the centre and the cube stays clear of the knuckle at 7 cm.
DEPTH = 0.007
#: ... and this far above the centre [m]. The fingers reach 1.9 cm below the
#: hand's centre line and the cube's centre is 2.25 cm above the stand, so
#: at the centre they cleared the stand by 3.5 mm and caught its front edge
#: (seen from above in Gazebo). 8 mm up, the cube swung ~30 deg about the
#: fingers as it was lifted (held above its centre of mass; 2 of 4 runs);
#: 4 mm up neither happened (2 of 2).
GRASP_UP = 0.004
#: Arm joint speed limit [deg/s]: swung out of the driving pose in 2.5 s
#: (~65 deg/s) the arm's reaction turned the whole robot 0.16 rad on its
#: wheels in Gazebo; at 5 s it did not move.
ARM_SPEED = 30.0
#: Straight-line motions are solved at points this far apart [m], and a
#: step needing a larger joint change than MAX_JUMP [deg] is refused (the
#: arm would swing rather than slide).
STEP = 0.01
MAX_JUMP = 12.0
#: Gripper angles [rad] between which the closing fingers have stopped on
#: something (the cube): not still open, not closed on nothing. Closed on
#: nothing they reach GRIPPER_CLOSE.
HOLDING = (-0.2, 0.4)
#: [m] A push shorter than this does not move the base: at the real base's
#: lowest speed it lasts ~0.1 s (measured in Gazebo: commanded 3-9 mm, the
#: cube stayed 0.209-0.219 m ahead for 5 pushes). Such a correction backs off
#: BACK_OFF first and comes in again by that much more.
MIN_PUSH = 0.03
BACK_OFF = 0.05
#: [m] The hand comes down onto the 'pre' pose from this far above it, in a
#: straight line. From ARMS_CLEAR (hands high and out), one joint-space move
#: to 'pre' swept the hand across the cube and knocked it 2 cm aside and 15
#: deg round (Gazebo ground truth) before the grasp. The path is traced UP
#: from 'pre' a centimetre at a time and reversed: one separate IK for the
#: point above failed for some cube positions 3 mm apart (model, from
#: ARMS_CLEAR); lower heights are tried if the full one does not trace.
ABOVE = 0.06
#: [m] How far beyond end_coords, along the approach, the held cube's centre
#: ends up. The fingers stop on the cube still 0.05-0.08 rad open (not
#: parallel) and squeeze it out towards their tips: in Gazebo (ground
#: truth) it moved 11-12 mm further away as they closed and another 2-6 mm
#: as it was lifted, and once put down it was 3-13 mm (mean 9 mm, 5 runs)
#: beyond where the hand put it. The hand therefore puts it down this much
#: nearer. The camera cannot measure it in each run instead: with the
#: fingers closed the hand hides every tag but the top one, which was then
#: 9-16 mm off.
HELD_AT = 0.009
#: [m] A cube seen more than this far from where it was picked, after being
#: put down, does not count as put back.
PLACE_TOLERANCE = 0.02


def look(ri, robot, K, detector, frames=3):
    """The cube's pose, from up to ``frames`` images (None if never seen)."""
    for _ in range(frames):
        rgb, _ = ri.image(newer_than=ri.now())
        robot.angle_vector(ri.angle_vector())
        cube = cube_pose(detector.detect(rgb, K, robot))
        if cube is not None:
            return cube
    return None


def grasp_frame(cube):
    """Hand rotation for a side grasp: x = approach, y = closing axis.

    The cube axis nearest to vertical is "up" (a cube fallen on its side
    still has one); of the other two, the one nearest the robot's +x is the
    approach, and the fingers close along the third.
    """
    axes = [cube.rotation[:, k] for k in range(3)]
    up = max(range(3), key=lambda k: abs(axes[k][2]))
    side = [k for k in range(3) if k != up]
    approach = max((axes[k] * np.sign(axes[k][0]) for k in side), key=lambda a: a[0])
    approach = approach - approach[2] * np.array([0.0, 0.0, 1.0])   # level
    approach /= np.linalg.norm(approach)
    closing = np.cross(np.array([0.0, 0.0, 1.0]), approach)
    return np.column_stack([approach, closing, np.cross(approach, closing)])


def straight(robot, arm, start, goal, rotation, step):
    """Angle vectors moving the hand in a straight line, ``step`` apart.

    Each point is solved from the previous solution alone, so consecutive
    poses are close and the hand, not only the joints, goes straight: one
    IK solution at each end, interpolated in joint space, swung the hand
    through an arc that hit the stand. The model must hold the pose at
    ``start``. None if a point fails or needs a jump of over MAX_JUMP.
    """
    from skrobot.coordinates import Coordinates
    n = max(1, int(math.ceil(np.linalg.norm(goal - start) / step)))
    path = []
    for k in range(1, n + 1):
        before = np.array([j.joint_angle() for j in arm.joint_list])
        point = start + (goal - start) * k / float(n)
        ok = arm.inverse_kinematics(Coordinates(pos=point, rot=rotation), rotation_axis=True,
                                    warn_if_fail=False)
        if ok is False or ok is None:
            return None
        after = np.array([j.joint_angle() for j in arm.joint_list])
        if np.degrees(np.max(np.abs(after - before))) > MAX_JUMP:
            return None
        path.append(robot.angle_vector().copy())
    return path


def plan(robot, arm, cube, pre_back, lift_up, start):
    """Arm motions to reach in to, grasp and lift the cube; None if out of reach.

    Two hand rotations hold the cube the same way (fingers swapped: the
    gripper is symmetric). For each, the grasp pose is solved first (from
    several start poses), the pose ``pre_back`` before it from that
    solution, and then straight lines from there in to the cube and up.

    Returns
    -------
    dict or None
        ``'above'``, ``'pre'``: angle vectors; ``'down'`` (from 'above' to
        'pre'), ``'in'``, ``'up'``: lists of angle vectors (straight lines
        down, in to the cube and up from it); ``'targets'``: the hand
        positions ``'above'``, ``'pre'``, ``'grasp'``, ``'lift'`` and
        ``'place'`` (to put the held cube down where it was: see HELD_AT);
        ``'rotation'``: the hand's rotation throughout.
    """
    R = grasp_frame(cube)
    grasp = cube.position + DEPTH * R[:, 0] + np.array([0.0, 0.0, GRASP_UP])
    targets = {'grasp': grasp, 'pre': grasp - pre_back * R[:, 0],
               'lift': grasp + np.array([0.0, 0.0, lift_up]),
               'place': grasp - (DEPTH + HELD_AT) * R[:, 0]}
    found = None
    for rot in (R, R.dot(np.diag([1.0, -1.0, -1.0]))):
        robot.angle_vector(start)
        if not solve_ik(robot, arm, grasp, rotation=rot):
            continue
        if not solve_ik(robot, arm, targets['pre'], rotation=rot):
            continue
        pre = robot.angle_vector().copy()
        path_down = None
        for height in (ABOVE, 0.75 * ABOVE, 0.5 * ABOVE):
            robot.angle_vector(pre)
            top = targets['pre'] + np.array([0.0, 0.0, height])
            path_up = straight(robot, arm, targets['pre'], top, rot, STEP)
            if path_up is not None:
                above, targets['above'] = path_up[-1], top
                path_down = path_up[::-1][1:] + [pre]
                break
        if path_down is None:
            continue
        robot.angle_vector(pre)
        path_in = straight(robot, arm, targets['pre'], grasp, rot, STEP)
        if path_in is None:
            continue
        path_up = straight(robot, arm, grasp, targets['lift'], rot, STEP)
        if path_up is None:
            continue
        found = {'above': above, 'down': path_down, 'pre': pre, 'in': path_in,
                 'up': path_up, 'targets': targets, 'rotation': rot}
        break
    robot.angle_vector(start)
    return found


def plan_put_down(robot, arm, poses):
    """Arm motions to put the held cube down where it was picked up.

    From the lift pose: across to above the place, straight down to it and,
    once the fingers are open, straight back out to the 'pre' pose (the
    place is on the line the hand came in along, short of the grasp pose),
    all with the hand's grasp rotation. From 'pre' the hand goes up along
    the plan's own path.

    Parameters
    ----------
    robot : skrobot.models.urdf.RobotModelFromURDF
        The robot model; left as it was.
    arm : skrobot.model.RobotModel
        The arm holding the cube.
    poses : dict
        The grasp plan (:func:`plan`).

    Returns
    -------
    dict or None
        ``'across'``, ``'down'``, ``'back'``: lists of angle vectors. None
        if a straight line fails.
    """
    rot, targets = poses['rotation'], poses['targets']
    over = targets['place'] + (targets['lift'] - targets['grasp'])
    saved = robot.angle_vector().copy()
    robot.angle_vector(poses['up'][-1])
    found = {}
    for name, start, goal in (('across', targets['lift'], over),
                              ('down', over, targets['place']),
                              ('back', targets['place'], targets['pre'])):
        found[name] = straight(robot, arm, start, goal, rot, STEP)
        if found[name] is None:
            found = None
            break
    robot.angle_vector(saved)
    return found


def choose_side(cube):
    """The arm for the cube: the right arm reaches y 0 to -0.12 m; the left mirrors it."""
    return 'larm' if cube.position[1] > 0.02 else 'rarm'


def drive_into_reach(ri, robot, K, detector, reachable, attempts=6, speed=0.08):
    """Forward/backward until the arm can grasp the cube. Returns the cube.

    The real base does not move below ~0.078 m/s, too fast to stop within a
    2 cm band by steering, so it moves in pushes, each as long as the
    remaining distance to the middle of REACH_X takes at that speed, and
    looks again after each, until the cube is within REACH_X. Stopping as soon
    as the IK solved (``reachable``) was tried: at the edge of the reach, the
    next look at the cube (12 mm off) left it out of reach again, so the
    middle is the goal and ``reachable`` only decides when the pushes run out.
    """
    goal = sum(REACH_X) / 2.0
    for _ in range(attempts):
        cube = look(ri, robot, K, detector, frames=10)
        if cube is None:
            raise SystemExit('lost the cube while driving to it (not in 10 images)')
        x = cube.position[0]
        print('cube {:.3f} m ahead (reach {:.3f}-{:.3f})'.format(x, *REACH_X))
        if REACH_X[0] <= x <= REACH_X[1]:
            return cube
        push = x - goal
        if abs(push) < MIN_PUSH:
            ri.go_velocity(-speed, 0.0, 0.0, duration=BACK_OFF / speed)
            ri.stop()
            ri.sleep(0.5)
            push += BACK_OFF
        ri.go_velocity(math.copysign(speed, push), 0.0, 0.0,
                       duration=abs(push) / speed)
        ri.stop()
        ri.sleep(1.0)   # let the base settle before looking
    cube = look(ri, robot, K, detector, frames=10)
    if cube is not None and reachable(cube):
        return cube
    raise SystemExit('could not bring the cube into reach ({:.3f}-{:.3f} m ahead) '
                     'in {} moves'.format(REACH_X[0], REACH_X[1], attempts))


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--tilt', type=float, default=35.0, help='head tilt, down [deg]')
    p.add_argument('--pre', type=float, default=0.05,
                   help='stop this far back before moving in [m]')
    p.add_argument('--lift', type=float, default=0.05, help='[m]')
    p.add_argument('--time', type=float, default=2.5, help='seconds per arm motion')
    p.add_argument('--open', type=float, default=GRIPPER_OPEN, help='gripper open [rad]')
    p.add_argument('--close', type=float, default=GRIPPER_CLOSE,
                   help='gripper closing command, past the cube [rad]')
    args = p.parse_args()

    robot, ri = connect()
    ri.servo_on()
    K, _ = ri.camera_info()
    set_head(robot, ri, 0.0, args.tilt)
    detector = TagDetector(TAG_CUBE_TAG, upscale=2)
    cube = look(ri, robot, K, detector)
    if cube is None:
        raise SystemExit('the tag cube is not in view: run approach_tag --standoff 0.17 first')
    print('cube at {} m, seen on faces {}'.format(
        np.round(cube.position, 3), sorted(t.id for t in cube.tags)))

    def reachable(cube):
        """Whether the arm on the cube's side can do the whole grasp from here."""
        av = robot.angle_vector().copy()
        side = choose_side(cube)
        ok = plan(robot, getattr(robot, side), cube, args.pre, args.lift, av) is not None
        robot.angle_vector(av)
        return ok

    if not reachable(cube):
        print('out of reach ({:.3f} m ahead): driving'.format(cube.position[0]))
        ri.tuck_arms(time=5.0, pose=ARMS_CLEAR)  # slowly: see ARM_SPEED
        cube = drive_into_reach(ri, robot, K, detector, reachable)
        print('cube at {} m'.format(np.round(cube.position, 3)))

    side = choose_side(cube)
    arm = getattr(robot, side)
    gripper = getattr(robot, '{}_gripper_joint'.format(side))
    arm_ctrl = '{}_arm_controller'.format('left' if side == 'larm' else 'right')
    grip_ctrl = '{}_gripper_controller'.format('left' if side == 'larm' else 'right')
    centre = cube.position
    home = robot.angle_vector().copy()
    poses = plan(robot, arm, cube, args.pre, args.lift, home)
    if poses is None:
        say(ri, '届かないのだ')
        raise SystemExit('IK: {} cannot reach in to, and lift, the cube at {} m with the '
                         'hand level. It should be {:.3f}-{:.3f} m ahead.'
                         .format(side, np.round(centre, 3), *REACH_X))

    def report(name, target, commanded):
        """How far the hand ended from ``target``; which joints fell short."""
        ri.sleep(0.3)   # let the servos settle
        robot.angle_vector(ri.angle_vector())
        err = np.linalg.norm(arm.end_coords.worldpos() - target)
        print('{}: {} hand {:.1f} mm from the target'.format(name, side, 1000 * err))
        if err > 0.01:
            # Which joint did not get there (blocked by the cube or the
            # stand, or too weak)?
            for j in arm.joint_list:
                print('  {:12s} commanded {:+7.1f} measured {:+7.1f} deg'.format(
                    j.name, np.degrees(commanded[j.name]), np.degrees(j.joint_angle())))

    def move_arm(avs, seconds, name=None, target=None):
        """Through ``avs`` (one angle vector or a list), no faster than ARM_SPEED.

        With ``target``, say how far the hand ended from it.
        """
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
        commanded = {j.name: j.joint_angle() for j in arm.joint_list}
        if len(avs) == 1:
            ri.angle_vector(avs[0], times[0], controller_type=arm_ctrl)
        else:
            ri.angle_vector_sequence(avs, times, controller_type=arm_ctrl)
        ri.wait_interpolation(arm_ctrl)
        if target is not None:
            report(name, target, commanded)

    def move_gripper(angle, seconds=1.0):
        robot.angle_vector(ri.angle_vector())
        gripper.joint_angle(angle)
        ri.angle_vector(robot.angle_vector(), seconds, controller_type=grip_ctrl)
        ri.wait_interpolation(grip_ctrl)
        robot.angle_vector(ri.angle_vector())
        return gripper.joint_angle()

    say(ri, '箱を持つのだ')
    # The fingers open only out here: opened in the driving pose they caught
    # on the body and held the arm back (shoulder stuck 70 deg short).
    move_gripper(0.0)
    move_arm(poses['above'], args.time, 'above', poses['targets']['above'])
    move_arm(poses['down'], 1.5, 'pre', poses['targets']['pre'])
    move_gripper(args.open)
    # Look again from here: the estimate made from farther away, and with the
    # robot maybe nudged by the swing, is refined before reaching in.
    again = look(ri, robot, K, detector)
    if again is None:
        print('cube not in view from the pre pose (the hand hides it): '
              'reaching in on the first estimate')
    else:
        shift = np.linalg.norm(again.position - cube.position)
        print('cube seen again, {:.1f} mm from the first estimate'.format(1000 * shift))
        if shift > 0.005:
            # From where the arm is now; failing that, from where it started
            # (the plan from the pre pose found no straight path in 1 of 8
            # runs where the one from the start did).
            replanned = (plan(robot, arm, again, args.pre, args.lift, poses['pre'])
                         or plan(robot, arm, again, args.pre, args.lift, home))
            if replanned is None:
                say(ri, '届かないのだ')
                raise SystemExit('IK: the cube is at {} m now, out of reach'.format(
                    np.round(again.position, 3)))
            poses, cube, centre = replanned, again, again.position
            move_arm(poses['pre'], 1.5, 'pre (again)', poses['targets']['pre'])
    move_arm(poses['in'], 2.0, 'in to the cube', poses['targets']['grasp'])
    move_gripper(args.close, 1.5)
    ri.sleep(0.5)
    robot.angle_vector(ri.angle_vector())
    closed = gripper.joint_angle()
    print('gripper stopped at {:+.2f} rad (closed on nothing: {:+.2f})'.format(
        closed, args.close))
    move_arm(poses['up'], 2.0, 'lift', poses['targets']['lift'])
    ri.sleep(0.5)
    robot.angle_vector(ri.angle_vector())
    holding = HOLDING[0] <= gripper.joint_angle() <= HOLDING[1]
    after = look(ri, robot, K, detector)
    if after is not None:
        rise = after.position[2] - centre[2]
        print('cube now at {} m: {:+.3f} m higher than before'.format(
            np.round(after.position, 3), rise))
        lifted = holding and rise > args.lift / 2
        how = 'gripper angle and camera'
    else:
        # The usual case: the lifted hand is between the camera and the cube.
        print('cube not in view now (the hand hides it): "lifted" is judged from '
              'the gripper angle alone -- the fingers stopped on something -- not '
              'from seeing the cube higher')
        lifted = holding
        how = 'gripper angle only'
    if lifted:
        say(ri, '持てたのだ')
        print('LIFTED ({})'.format(how))
    else:
        say(ri, '持てなかったのだ')
        angle = gripper.joint_angle()
        if holding:
            why = ''
        elif abs(angle - args.close) < abs(angle - args.open):
            why = ' (closed on nothing: the cube slipped out or was missed)'
        else:
            why = ' (the gripper did not close: blocked?)'
        print('NOT LIFTED: gripper {:+.2f} rad{}'.format(angle, why))
    ri.sleep(1.0)
    put = plan_put_down(robot, arm, poses)
    if put is None:
        print('IK: no straight path to put the cube down where it was: putting it '
              'down at the grasp pose, {:.0f} mm further away'.format(
                  1000 * (DEPTH + HELD_AT)))
        put = {'down': poses['up'][::-1][1:] + [poses['in'][-1]],
               'back': poses['in'][::-1][1:] + [poses['pre']]}
        place = poses['targets']['grasp']
    else:
        move_arm(put['across'], 1.0)
        place = poses['targets']['place']
    move_arm(put['down'], 2.0, 'put down', place)
    move_gripper(args.open)   # let go only once the arm has stopped
    move_arm(put['back'], 2.0)
    move_gripper(0.0)   # before folding back, for the same reason
    move_arm(poses['down'][::-1][1:] + [poses['above']], 1.5)   # up, clear of the cube
    move_arm(home, args.time)
    # Is the cube back where it was? From here the camera sees it as it did
    # at the start (the hand is out of the way).
    placed = look(ri, robot, K, detector, frames=10)
    if placed is None:
        put_back = False
        print('NOT PUT BACK: the cube is not in view after putting it down -- it may '
              'have fallen off the stand')
    else:
        off = placed.position - centre
        print('cube now at {} m, {:.1f} mm from where it was picked up {}'.format(
            np.round(placed.position, 3), 1000 * np.linalg.norm(off),
            np.round(1000 * off, 1)))
        put_back = np.linalg.norm(off) <= PLACE_TOLERANCE
        if put_back:
            print('PUT BACK')
        elif off[2] < -TAG_CUBE_EDGE / 2:
            print('NOT PUT BACK: the cube is {:.0f} mm lower -- it fell off the stand'
                  .format(-1000 * off[2]))
        else:
            print('NOT PUT BACK: more than {:.0f} mm from where it was picked up'
                  .format(1000 * PLACE_TOLERANCE))
    if lifted and put_back:
        say(ri, '元の場所に置いたのだ')
    elif lifted:
        say(ri, '元の場所に置けなかったのだ')
    if not (lifted and put_back):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
