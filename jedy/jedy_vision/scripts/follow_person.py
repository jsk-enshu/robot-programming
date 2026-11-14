#!/usr/bin/env python3
"""Turn towards the nearest person and drive up to them, then stop.

track_box with a person instead of a red box: YOLO finds the people, the
biggest box is the one to follow, the base turns to put it in the middle of
the picture and drives until the depth inside the box is ``--distance``.

    ros2 launch jedy_bringup jedy_gazebo.launch.py person:=true   # terminal 1
    ros2 run jedy_vision follow_person                            # terminal 2
    ros2 run jedy_vision follow_person --distance 1.5 --keep      # keep following

Like track_box, a detection older than ``--max-age`` seconds is not used and
the base stops. Until someone is seen for the first time it turns on the
spot to look for them (`--search 0` to just wait).

The real base does not turn on the spot slower than ~0.83 rad/s, nor
forward slower than ~0.078 m/s (docs). So every turn on the spot is a burst
at ``--min-turn`` as long as the angle to the person takes -- at least
``--min-burst`` seconds (a shorter one may not move the real base), at most
MAX_BURST -- then a stop and a fresh look; and a forward speed below
``--min-speed`` is not sent at all. The robot drives only straight ahead,
and only while the person is within the deadband (``--tolerance``, and at
least 0.6 of the shortest burst, so one burst past them lands inside it
instead of swinging back and forth).

It stops and says why when it cannot move:

- driving forward for ``--stuck`` seconds without the person coming
  STUCK_PROGRESS closer (in Gazebo's demo stage, demo:=true person:=true,
  it ran onto the stand and its wheels spun; the wheels' own odometry
  would not tell, it counts the spinning);
- NO_TURN_LIMIT turns in a row after which the person is where they were
  in the picture.
"""

import argparse
import math

import numpy as np

from jedy_skrobot import connect
from jedy_vision import (FLOOR_Z, PersonDetector, box_distance, pixel_on_floor,
                         pixel_on_plane)

#: Seconds (robot clock) with nobody ever seen before printing a hint.
NO_PERSON_HINT_S = 3.0
#: [rad] Largest turn on the spot between two looks: a person at the edge of
#: the picture (or a bad box) must not swing the robot far past them.
MAX_BURST = math.radians(25.0)
#: [s] Wait after a turn before the next look (a sharp image of a still robot).
SETTLE_S = 0.3
#: [m] Driving forward for ``--stuck`` seconds must bring the person at least
#: this much closer. Seen this much further away, they stepped back: the
#: count starts again from there.
STUCK_PROGRESS = 0.1
#: Turns in a row that may leave the person's direction unchanged (moved by
#: less than NO_TURN_FRACTION of the turn) before giving up.
NO_TURN_LIMIT = 4
NO_TURN_FRACTION = 0.3


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--distance', type=float, default=1.0,
                   help='stop this far from the person [m]')
    p.add_argument('--speed', type=float, default=0.15, help='max forward [m/s]')
    p.add_argument('--min-turn', type=float, default=0.9,
                   help='[rad/s] turning on the spot (the real base: >= 0.83)')
    p.add_argument('--tolerance', type=float, default=0.1,
                   help='centred when |error| (-1..1 across the image) is below this; '
                        'otherwise the robot stops and turns towards the person')
    p.add_argument('--min-burst', type=float, default=0.25,
                   help='[s] shortest turn on the spot (a shorter command may not '
                        'move the real base)')
    p.add_argument('--min-speed', type=float, default=0.08,
                   help='[m/s]: the real base does not move slower (docs)')
    p.add_argument('--stuck', type=float, default=5.0,
                   help='[s] stop if driving forward this long brings the person '
                        'less than {:.2f} m closer'.format(STUCK_PROGRESS))
    p.add_argument('--keep', action='store_true',
                   help='keep following instead of stopping when reached')
    p.add_argument('--max-age', type=float, default=1.0,
                   help='ignore detections older than this [s]')
    p.add_argument('--tilt', type=float, default=-25.0,
                   help='head tilt while following [deg], negative = up. The '
                        'camera is only 0.3 m above the floor: looking straight '
                        'ahead it sees just the legs of someone 1 m away, and '
                        'YOLO does not call legs a person.')
    p.add_argument('--height', type=float, default=1.6,
                   help="assumed person height [m], for the distance when the feet "
                        "are out of view and there is no depth")
    p.add_argument('--search', type=float, default=30.0,
                   help='[deg] turn on the spot this far between looks until someone '
                        'is seen for the first time (0: just wait). Once a person '
                        'has been seen and is lost, the robot stops instead.')
    p.add_argument('--timeout', type=float, default=60.0, help='[s], robot clock')
    args = p.parse_args()

    # Before connect(): connected, the model's first (loading) run is ~8x slower.
    detector = PersonDetector()
    robot, ri = connect()
    if not ri.has_camera():
        raise SystemExit('no head camera image (render_sensors:=true in Gazebo).')
    K, _ = ri.camera_info()
    has_depth = ri.count_publishers('/camera/depth/image_rect_raw') > 0
    ri.tuck_arms()  # arms up before driving: hanging, they drag on the floor
    robot.angle_vector(ri.angle_vector())
    robot.head_joint0.joint_angle(0.0)
    robot.head_joint1.joint_angle(math.radians(args.tilt))
    ri.angle_vector(robot.angle_vector(), 1.0, controller_type='head_controller')
    ri.wait_interpolation('head_controller')
    start = ri.now()
    reason, hinted, seen = 'timeout', False, False
    searched = 0.0     # [rad] turned while looking for someone
    driven = 0.0       # [s] driven forward since the person last came closer
    moving_since = None  # when the last forward command was sent
    closest = None     # [m] the distance then
    newer_than = None  # only use images taken after the last turn
    turned = None      # (bearing before, angle) of the last turn at the person
    no_turn = 0        # turns in a row that did not change their direction
    min_angle = args.min_turn * args.min_burst

    def turn(angle):
        """Turn on the spot by ``angle`` [rad] at --min-turn, stop and settle.

        Returns the time after which an image shows the robot still.
        """
        ri.go_velocity(0.0, 0.0, math.copysign(args.min_turn, angle),
                       duration=abs(angle) / args.min_turn)
        ri.sleep(SETTLE_S)
        return ri.now()
    try:
        while ri.now() - start < args.timeout:
            if moving_since is not None:
                driven += ri.now() - moving_since
                moving_since = None
            else:
                # Not driven forward last cycle (stopped, turned, lost the
                # person or their distance): the stuck count starts again.
                driven, closest = 0.0, None
            rgb, stamp = ri.image(newer_than=newer_than)
            people = detector.detect(rgb)
            vx = 0.0
            if people and ri.now() - stamp < args.max_age:
                hinted, seen = True, True
                person = people[0]
                cx, _ = person.center
                error = (cx - rgb.shape[1] / 2.0) / (rgb.shape[1] / 2.0)
                # Depth inside the box where there is any (a D405 has none
                # beyond 0.5 m); else where the feet meet the floor (colour
                # only, any range: the camera pose comes from the model).
                dist, how = None, 'depth'
                if has_depth:
                    depth, _ = ri.depth_image()
                    dist = box_distance(depth, person.box)
                if dist is None:
                    robot.angle_vector(ri.angle_vector())
                    feet = pixel_on_floor(robot, K, cx, person.box[3], FLOOR_Z)
                    if feet is not None and person.box[3] < rgb.shape[0] - 2:
                        dist, how = float(np.hypot(feet[0], feet[1])), 'floor'
                    elif person.box[1] > 2:
                        # Feet out of view (the head looks up): assume the top
                        # of the box is a head --height above the floor.
                        head = pixel_on_plane(robot, K, cx, person.box[1],
                                              FLOOR_Z + args.height)
                        if head is not None:
                            dist, how = float(np.hypot(head[0], head[1])), 'head height'
                # Direction of the box's centre (left positive); centred
                # within the deadband; else one burst towards it.
                bearing = -math.atan2(cx - K[0, 2], K[0, 0])
                deadband = max(math.atan(args.tolerance * K[0, 2] / K[0, 0]),
                               0.6 * min_angle)
                centred = abs(bearing) <= deadband
                burst = math.copysign(min(max(abs(bearing), min_angle), MAX_BURST),
                                      bearing)
                if turned is not None:
                    before, angle = turned
                    turned = None
                    if abs(before - bearing) < NO_TURN_FRACTION * abs(angle):
                        no_turn += 1
                    else:
                        no_turn = 0
                    if no_turn >= NO_TURN_LIMIT:
                        reason = 'not turning'
                        break
                if centred and dist is not None:
                    # Slow down on the way in; never back up.
                    vx = float(np.clip(0.5 * (dist - args.distance), 0.0, args.speed))
                    if 0.0 < vx < args.min_speed:
                        # The real base stands still below this: drive at it
                        # while still more than 5 cm off, else stop.
                        vx = args.min_speed if dist - args.distance > 0.05 else 0.0
                if not centred:
                    action = 'turn {:+.0f} deg'.format(math.degrees(burst))
                elif vx > 0.0:
                    action = 'forward {:.2f} m/s'.format(vx)
                else:
                    action = 'stop'
                print('person at x={:.0f} ({:+.2f}, {:+.0f} deg), {} -> {}'.format(
                    cx, error, math.degrees(bearing),
                    'distance unknown' if dist is None
                    else '{:.2f} m ({})'.format(dist, how), action))
                reached = dist is not None and dist <= args.distance + 0.05 and centred
                if reached and not args.keep:
                    reason = 'reached'
                    break
                if not centred:
                    turned = (bearing, burst)
                    newer_than = turn(burst)
                    continue
                if vx > 0.0:
                    if (closest is None or closest - dist >= STUCK_PROGRESS
                            or dist - closest >= STUCK_PROGRESS):
                        driven, closest = 0.0, dist
                    elif driven > args.stuck:
                        reason = 'stuck'
                        break
            elif not seen and args.search > 0.0:
                turned = None
                # Nobody yet: look around, a burst at a time.
                print('nobody yet -> turn {:+.0f} deg to look for someone'.format(args.search))
                newer_than = turn(math.radians(args.search))
                searched += math.radians(args.search)
                if not hinted and searched >= 2.0 * math.pi:
                    hinted = True
                    print('hint: 1 周回っても人が見えません. Gazebo は '
                          'person:=true で起動しましたか? `ros2 run jedy_vision '
                          'detect_people` で今の画像に人が写っているか確かめられます.')
                continue
            else:
                turned = None
                print('nobody -> stop')
                if not hinted and ri.now() - start > NO_PERSON_HINT_S:
                    hinted = True
                    print('hint: {:.0f} 秒間, 人が一度も見えていません. Gazebo は '
                          'person:=true で起動しましたか? `ros2 run jedy_vision '
                          'detect_people` で今の画像に人が写っているか確かめられます.'
                          .format(NO_PERSON_HINT_S))
            ri.go_velocity(vx, 0.0, 0.0)
            if vx > 0.0:
                moving_since = ri.now()
            ri.sleep(0.05)
    finally:
        ri.stop()
    print('stopped by {} after {:.1f} s'.format(reason, ri.now() - start))
    if reason == 'not turning':
        raise SystemExit(
            '{} 回続けて人の方へ回ろうとしたのに, 向きが変わりませんでした. 車輪が'
            '空回りしているか, 台車が何かに引っかかっています. ロボットのまわりを見てください.\n'
            'Turned towards the person {} times in a row and their direction did '
            'not change: the wheels are slipping or the base is caught on something. '
            'Look around the robot.'.format(NO_TURN_LIMIT, NO_TURN_LIMIT))
    if reason == 'stuck':
        raise SystemExit(
            '前に {:.0f} 秒進もうとしたのに, 人までの距離が {:.2f} m から {:.2f} m 以上'
            '縮まりませんでした. 何かにぶつかって進めない (車輪が空回りしている) か, '
            '人が離れていっています. ロボットのまわりを見てください.\n'
            'Drove forward for {:.0f} s but the person got no closer than {:.2f} m '
            '(less than {:.2f} m of progress): the robot is blocked (wheels '
            'spinning?) or the person is moving away. Look around the robot.'.format(
                args.stuck, closest, STUCK_PROGRESS, args.stuck, closest,
                STUCK_PROGRESS))


if __name__ == '__main__':
    main()
