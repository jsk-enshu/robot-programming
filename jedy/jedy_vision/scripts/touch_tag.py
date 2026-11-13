#!/usr/bin/env python3
"""Touch an AprilTag with the hand on its side (inverse kinematics).

    ros2 run jedy_vision approach_tag     # first: stand 0.2 m in front of the stand
    ros2 run jedy_vision touch_tag        # the cube's front tag (id 1)
    ros2 run jedy_vision touch_tag --id 0 # the stand's tag: too low on the
                                          # demo stage, IK says so

The tag's pose (colour image, tag_pose) is a point in the robot's base frame,
so it is directly an IK target: the hand goes 1 cm in front of the tag's
centre, waits, and comes back. Close up the D405's depth works too, so the
distance is also read from the depth image and printed next to the
colour-only estimate: two sensors, one answer (or not).

The hand cannot reach the floor or farther than ~0.25 m ahead; if IK fails
the program says so instead of moving.
"""

import argparse

import numpy as np

from jedy_skrobot import connect
from jedy_vision import DEMO_TAG_SIZES, TagDetector, say, set_head, solve_ik


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--id', type=int, default=1,
                   help='tag id (demo stage: 1 = the cube\'s front face)')
    p.add_argument('--size', type=float, default=None,
                   help='tag edge [m] (default: the demo stage\'s, by id)')
    p.add_argument('--gap', type=float, default=0.01, help='stop this far before the face [m]')
    p.add_argument('--pre', type=float, default=0.06,
                   help='first stop this much farther out, then move in [m]')
    p.add_argument('--tilt', type=float, default=30.0, help='head tilt, down [deg]')
    # 5 s out of the driving pose: swung out in 2.5 s (~65 deg/s) the arm's
    # reaction turned the whole robot on its wheels in Gazebo (grasp_tag).
    p.add_argument('--time', type=float, default=5.0, help='seconds per arm motion')
    args = p.parse_args()

    robot, ri = connect()
    ri.servo_on()
    K, _ = ri.camera_info()
    set_head(robot, ri, 0.0, args.tilt)
    rgb, _ = ri.image(newer_than=ri.now())
    robot.angle_vector(ri.angle_vector())
    size = DEMO_TAG_SIZES if args.size is None else args.size
    tags = [t for t in TagDetector(size, upscale=2).detect(rgb, K, robot) if t.id == args.id]
    if not tags:
        raise SystemExit('tag {} not in view: run approach_tag first'.format(args.id))
    tag = tags[0]
    target = tag.position + args.gap * tag.normal
    cam = robot.camera_color_optical_frame.worldpos()
    print('tag centre {} m (colour, {:.3f} m from the camera)'.format(
        np.round(tag.position, 3), np.linalg.norm(tag.position - cam)))
    if ri.count_publishers('/camera/depth/image_rect_raw') > 0:
        depth, _ = ri.depth_image()
        u, v = (int(x) for x in tag.corners.mean(axis=0))
        d = depth[v, u]
        print('depth at the tag centre: {}'.format(
            'no reading (out of the D405 range?)' if not np.isfinite(d) else '{:.3f} m'.format(d)))

    side = 'larm' if tag.position[1] > 0.03 else 'rarm'
    arm = getattr(robot, side)
    home = robot.angle_vector().copy()
    # Two steps: first to a point in front of the tag, then straight in.
    # Swinging the arm straight from the folded pose to the face drives it
    # into the box (the shoulder stopped at -2 of -30 deg, 12 cm short).
    approach = tag.position + (args.gap + args.pre) * tag.normal
    poses = []
    for point in (approach, target):
        if not solve_ik(robot, arm, point):
            say(ri, '届かないのだ')
            raise SystemExit('IK: {} cannot reach {} m (reachable: about 0.15-0.25 m '
                             'ahead at this height). Run approach_tag again.'.format(
                                 side, np.round(point, 3)))
        poses.append(robot.angle_vector().copy())
    commanded = {j.name: j.joint_angle() for j in arm.joint_list}
    say(ri, 'タッチするのだ')
    ri.angle_vector(poses[0], args.time)
    ri.wait_interpolation()
    ri.angle_vector(poses[1], 1.5)
    ri.wait_interpolation()
    robot.angle_vector(ri.angle_vector())
    err = np.linalg.norm(arm.end_coords.worldpos() - target)
    print('{} hand {:.1f} mm from the target'.format(side, 1000 * err))
    if err > 0.02:
        # Which joint did not get there (blocked by the box, or a limit)?
        measured = {j.name: j.joint_angle() for j in arm.joint_list}
        for j in arm.joint_list:
            print('  {:12s} commanded {:+7.1f} measured {:+7.1f} deg'.format(
                j.name, np.degrees(commanded[j.name]), np.degrees(measured[j.name])))
    ri.sleep(1.0)
    ri.angle_vector(poses[0], 1.5)
    ri.wait_interpolation()
    ri.angle_vector(home, args.time)
    ri.wait_interpolation()


if __name__ == '__main__':
    main()
