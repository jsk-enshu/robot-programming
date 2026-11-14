#!/usr/bin/env python3
"""Drive up to an AprilTag and stop square in front of it.

    ros2 launch jedy_bringup jedy_gazebo.launch.py demo:=true camera:=d405
    ros2 run jedy_vision approach_tag              # tag 0, stop 0.20 m from its face
    ros2 run jedy_vision approach_tag --standoff 0.3

Every cycle the tag is seen again (tag_pose: colour only, so it works at any
range) and the goal -- ``standoff`` in front of the tag, facing it -- is
expressed in the robot's own frame. The base is driven with forward speed and
turning only (no sideways motion: Gazebo's wheels have no rollers, so the
same code runs everywhere), with the textbook pose controller in polar
coordinates:

    rho   = distance to the goal
    alpha = direction of the goal, seen from the robot
    beta  = goal heading - alpha
    v = K_RHO * rho,   w = K_ALPHA * alpha + K_BETA * beta
    (stable for K_ALPHA > K_RHO > 0 > K_BETA)

It is closed loop, so wheel slip is corrected. If the tag is lost the base
turns towards the side it was last seen on -- except close to the goal: the
stand's tag (0) slides down the image as the robot comes in and goes behind
the tucked hands' fingertips about 4 cm short of the 0.20 m standoff (seen in
Gazebo's camera image). Lost within ``BLIND_FINISH`` of the goal, the robot
finishes on the last estimate instead (turn towards the goal, drive the
distance, turn to face the tag), which at that range is good to about a
centimetre.

The real base does not move below ~0.078 m/s forward or ~0.83 rad/s turning
on the spot (docs), so the forward command is raised to ``--min-speed`` and
every turn on the spot (searching, and the last turn to face the tag) runs at
``--min-turn``. The last turn is too small to steer at that speed, so it is a
timed burst (angle / speed, at most ``MAX_BURST`` per look: one bad estimate
of a half-hidden tag must not swing the robot away), then a stop and a fresh
look.
"""

import argparse
import math

from jedy_skrobot import connect
from jedy_vision import ARMS_CLEAR, DEMO_TAG_SIZE, TagDetector, say, set_head


#: Pose-controller gains (see the module docstring).
K_RHO, K_ALPHA, K_BETA = 0.5, 1.2, -0.4
#: [rad] Largest turn on the spot between two looks at the tag.
MAX_BURST = math.radians(10.0)
#: [m] Lost this close to the goal, finish on the last estimate (see above).
BLIND_FINISH = 0.12


def limit(v, vmax, vmin=0.0):
    """Clamp |v| to vmax and lift it to vmin (unless it is zero)."""
    if v == 0.0:
        return 0.0
    return math.copysign(min(max(abs(v), vmin), vmax), v)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--id', type=int, default=0, help='tag id')
    p.add_argument('--size', type=float, default=DEMO_TAG_SIZE, help='tag edge [m]')
    p.add_argument('--standoff', type=float, default=0.20,
                   help='stop this far from the tag face [m] (0.20: within arm reach)')
    p.add_argument('--tilt', type=float, default=30.0, help='head tilt, down [deg]')
    p.add_argument('--tolerance', type=float, default=0.02, help='[m]')
    p.add_argument('--min-speed', type=float, default=0.08, help='[m/s]')
    p.add_argument('--min-turn', type=float, default=0.9,
                   help='[rad/s] turning on the spot (the real base: >= 0.83)')
    p.add_argument('--timeout', type=float, default=90.0, help='[s], robot clock')
    args = p.parse_args()

    robot, ri = connect()
    K, _ = ri.camera_info()
    # Arms up before driving (hanging, they drag on the floor), and clear of
    # the stand and of the camera's view of its tag.
    ri.tuck_arms(pose=ARMS_CLEAR)
    set_head(robot, ri, 0.0, args.tilt)
    detector = TagDetector(args.size)
    start, last_seen, last_side, reason = ri.now(), ri.now(), 1.0, 'timeout'
    last = None   # (rho, alpha, goal_heading, backwards) of the last look

    def turn(angle):
        if abs(angle) > math.radians(2.0):
            ri.go_velocity(0.0, 0.0, math.copysign(args.min_turn, angle),
                           duration=abs(angle) / args.min_turn)
    try:
        while ri.now() - start < args.timeout:
            rgb, _ = ri.image(newer_than=ri.now() - 0.05)
            robot.angle_vector(ri.angle_vector())
            tags = [t for t in detector.detect(rgb, K, robot) if t.id == args.id]
            if not tags:
                lost = ri.now() - last_seen
                if last is not None and last[0] < BLIND_FINISH and lost < 1.0:
                    rho, alpha, heading, backwards = last
                    print('tag {} lost {:.0f} cm from the goal: finishing on the last '
                          'estimate'.format(args.id, 100 * rho))
                    turn(alpha)
                    ri.go_velocity(args.min_speed * (-1.0 if backwards else 1.0), 0.0, 0.0,
                                   duration=rho / args.min_speed)
                    turn(heading - alpha)
                    reason = 'reached (the last {:.0f} cm blind)'.format(100 * rho)
                    break
                print('tag {} not in view ({:.1f} s)'.format(args.id, lost))
                ri.go_velocity(0.0, 0.0,
                               args.min_turn * last_side if lost > 1.0 else 0.0)
                ri.sleep(0.1)
                continue
            last_seen = ri.now()
            tag = tags[0]
            last_side = 1.0 if tag.position[1] >= 0 else -1.0
            goal = tag.position + args.standoff * tag.normal  # in front of the face
            goal_heading = math.atan2(-tag.normal[1], -tag.normal[0])  # face the tag
            rho = math.hypot(goal[0], goal[1])
            alpha = math.atan2(goal[1], goal[0])
            backwards = abs(alpha) > math.pi / 2  # overshot: back up instead
            if backwards:
                alpha = math.atan2(-goal[1], -goal[0])
            beta = goal_heading - alpha
            last = (rho, alpha, goal_heading, backwards)
            print('rho {:.3f} m  alpha {:+5.1f}  heading error {:+5.1f} deg'.format(
                rho, math.degrees(alpha), math.degrees(goal_heading)))
            if rho < args.tolerance and abs(goal_heading) < math.radians(3.0):
                reason = 'reached'
                break
            if rho < args.tolerance:   # there: only turn to face the tag
                # Not ``turn``: that name is the function above, still
                # needed if the tag is lost after this burst.
                burst = math.copysign(min(abs(goal_heading), MAX_BURST), goal_heading)
                ri.go_velocity(0.0, 0.0, math.copysign(args.min_turn, burst),
                               duration=abs(burst) / args.min_turn)
                # The robot has turned by ``burst``: if the tag is lost now,
                # the blind finish must not turn by it again.
                last = (rho, alpha - burst, goal_heading - burst, backwards)
                ri.sleep(0.3)  # let it settle before the next look
                continue
            else:
                v = limit(K_RHO * rho, 0.12, args.min_speed) * (-1.0 if backwards else 1.0)
                w = limit(K_ALPHA * alpha + K_BETA * beta, 0.5)
            ri.go_velocity(v, 0.0, w)
            ri.sleep(0.1)
    finally:
        ri.stop()
    print('stopped by {} after {:.1f} s'.format(reason, ri.now() - start))
    if reason.startswith('reached'):
        say(ri, 'タグの前に着いたのだ')
        ri.sleep(0.3)


if __name__ == '__main__':
    main()
