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
"""

import argparse
import math

import numpy as np

from jedy_skrobot import connect
from jedy_vision import (FLOOR_Z, PersonDetector, box_distance, pixel_on_floor,
                         pixel_on_plane)

#: Seconds (robot clock) with nobody ever seen before printing a hint.
NO_PERSON_HINT_S = 3.0


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--distance', type=float, default=1.0,
                   help='stop this far from the person [m]')
    p.add_argument('--speed', type=float, default=0.15, help='max forward [m/s]')
    p.add_argument('--turn', type=float, default=0.4, help='max turn [rad/s]')
    p.add_argument('--tolerance', type=float, default=0.1,
                   help='centred when |error| (-1..1 across the image) is below this')
    p.add_argument('--min-speed', type=float, default=0.08,
                   help='[m/s]: the real base does not move slower (docs)')
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
    p.add_argument('--search', type=float, default=0.35,
                   help='turn on the spot at this speed [rad/s] until someone is '
                        'seen for the first time (0: just wait). Once a person '
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
    try:
        while ri.now() - start < args.timeout:
            rgb, stamp = ri.image()
            people = detector.detect(rgb)
            vx, vtheta = 0.0, 0.0
            if people and ri.now() - stamp < args.max_age:
                hinted, seen = True, True
                person = people[0]
                cx, _ = person.center
                error = (cx - rgb.shape[1] / 2.0) / (rgb.shape[1] / 2.0)
                if abs(error) > args.tolerance:
                    vtheta = -float(np.sign(error)) * min(1.0, 2.0 * abs(error)) * args.turn
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
                if dist is not None and abs(error) < 3 * args.tolerance:
                    # Slow down on the way in; never back up.
                    vx = float(np.clip(0.5 * (dist - args.distance), 0.0, args.speed))
                    if 0.0 < vx < args.min_speed and dist - args.distance > 0.05:
                        vx = args.min_speed  # the real base stands still below this
                print('person at x={:.0f} ({:+.2f}), {} -> forward {:.2f} m/s, '
                      'turn {:+.2f} rad/s'.format(
                          cx, error,
                          'distance unknown' if dist is None
                          else '{:.2f} m ({})'.format(dist, how),
                          vx, vtheta))
                reached = (dist is not None and dist <= args.distance + 0.05
                           and abs(error) <= args.tolerance)
                if reached and not args.keep:
                    reason = 'reached'
                    break
            elif not seen and args.search > 0.0:
                # Nobody yet: look around (one full turn takes 2*pi/search s).
                print('nobody yet -> turn {:+.2f} rad/s to look for someone'.format(args.search))
                vtheta = args.search
                if not hinted and ri.now() - start > 2.0 * math.pi / args.search:
                    hinted = True
                    print('hint: 1 周回っても人が見えません. Gazebo は '
                          'person:=true で起動しましたか? `ros2 run jedy_vision '
                          'detect_people` で今の画像に人が写っているか確かめられます.')
            else:
                print('nobody -> stop')
                if not hinted and ri.now() - start > NO_PERSON_HINT_S:
                    hinted = True
                    print('hint: {:.0f} 秒間, 人が一度も見えていません. Gazebo は '
                          'person:=true で起動しましたか? `ros2 run jedy_vision '
                          'detect_people` で今の画像に人が写っているか確かめられます.'
                          .format(NO_PERSON_HINT_S))
            ri.go_velocity(vx, 0.0, vtheta)
            ri.sleep(0.05)
    finally:
        ri.stop()
    print('stopped by {} after {:.1f} s'.format(reason, ri.now() - start))


if __name__ == '__main__':
    main()
