#!/usr/bin/env python3
"""Turn the base towards a coloured box seen by the head camera.

The scikit-robot version of jedy_bringup/scripts/track_box_to_cmd_vel_1.py
from the 2025 text, which turned towards opencv_apps' camshift box. Here the
box is found in the image directly with OpenCV (colour threshold, the largest
blob), so the whole exercise is one Python file:

    ros2 run jedy_skrobot track_box                       # a red box
    ros2 run jedy_skrobot track_box --until-centered      # stop once it faces it

In Gazebo, start with a box to look at:

    ros2 launch jedy_bringup jedy_gazebo.launch.py box:=true

Like the 2025 node, a detection older than ``--max-age`` seconds is not used
and the base stops: if the box leaves the picture the robot does not keep
turning on an old answer.
"""

import argparse

import cv2
import numpy as np

from jedy_skrobot import connect

#: HSV ranges of "red" for OpenCV (hue 0-180 wraps around red).
RED = [((0, 120, 70), (10, 255, 255)), ((170, 120, 70), (180, 255, 255))]
#: Seconds (robot clock) with nothing red ever seen before printing the
#: "did you start the world with the box?" hint.
NO_BOX_HINT_S = 3.0


def find_box(rgb, ranges=RED, min_area=100):
    """The largest blob of the colour in ``ranges``.

    Parameters
    ----------
    rgb : numpy.ndarray
        ``(H, W, 3)`` uint8 RGB.
    ranges : list of tuple
        ``[(hsv_low, hsv_high), ...]``, OpenCV's HSV scale.
    min_area : int
        Blobs smaller than this many pixels are noise.

    Returns
    -------
    tuple or None
        ``(cx, cy, w, h)`` in pixels, or None if there is no such blob.
    """
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for low, high in ranges:
        mask |= cv2.inRange(hsv, np.array(low), np.array(high))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < min_area:
        return None
    x, y, w, h = cv2.boundingRect(largest)
    return x + w / 2.0, y + h / 2.0, w, h


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--turn', type=float, default=1.0,
                   help='largest turning speed [rad/s]')
    p.add_argument('--timeout', type=float, default=30.0,
                   help='stop after this many seconds (robot clock)')
    p.add_argument('--min-turn', type=float, default=0.83,
                   help='slowest turn that is sent [rad/s]: the real base '
                        'does not turn below about 0.83 rad/s')
    p.add_argument('--tolerance', type=float, default=0.15,
                   help='how far from the middle counts as centred (1 = the '
                        'edge of the picture); at --min-turn the base turns '
                        'a few degrees per loop, so too small a band makes '
                        'it swing past the box and back')
    p.add_argument('--max-age', type=float, default=1.0,
                   help='ignore a detection older than this [s]')
    p.add_argument('--until-centered', action='store_true',
                   help='stop once the box is in the middle of the image')
    p.add_argument('--save', default=None,
                   help='write the last image, with the box drawn, here')
    args = p.parse_args()

    _, ri = connect()
    ri.tuck_arms()  # arms up before driving: hanging, they drag on the floor
    if not ri.has_camera():
        print('this robot has no head camera image (nobody publishes '
              '/camera/color/image_rect_raw). In Gazebo use '
              'render_sensors:=true; on the robot start the camera node.')
        return
    has_depth = ri.count_publishers('/camera/depth/image_rect_raw') > 0

    start = ri.now()
    rgb, box, reason = None, None, 'timeout'
    hinted = False
    try:
        while ri.now() - start < args.timeout:
            rgb, stamp = ri.image()
            box = find_box(rgb)
            vtheta = 0.0
            if box is not None and ri.now() - stamp < args.max_age:
                cx, cy, w, h = box
                # -1 at the left edge of the picture, +1 at the right.
                error = (cx - rgb.shape[1] / 2.0) / (rgb.shape[1] / 2.0)
                # Box on the left -> turn left (counter-clockwise, +z),
                # faster the further off it is, never slower than --min-turn.
                if abs(error) > args.tolerance:
                    speed = max(args.min_turn,
                                min(1.0, 2.0 * abs(error)) * args.turn)
                    vtheta = -float(np.sign(error)) * speed
                distance = ''
                if has_depth:
                    depth, _ = ri.depth_image()
                    patch = depth[int(cy) - 2:int(cy) + 3,
                                  int(cx) - 2:int(cx) + 3]
                    if np.isfinite(patch).any():
                        distance = ', {:.2f} m away'.format(
                            float(np.nanmedian(patch)))
                print('box at x={:.0f} ({:+.2f}), {}x{} px{} -> turn {:+.2f} '
                      'rad/s'.format(cx, error, w, h, distance, vtheta))
                if args.until_centered and abs(error) <= args.tolerance:
                    reason = 'centered'
                    break
            else:
                print('no box -> stop')
                # Nothing red since the start: almost always the world has no
                # box, not a vision bug. Say so once instead of only looping.
                if not hinted and ri.now() - start > NO_BOX_HINT_S:
                    hinted = True
                    print('hint: {:.0f} 秒間, 赤い箱が一度も見えていません. '
                          'このプログラムは見えない箱を探して回りはしません. '
                          '箱が画面の外にある (台車を動かしたあと) なら, '
                          'teleop で箱の方に向けるか Gazebo を起動し直す. '
                          'Gazebo は box:=true で起動しましたか? '
                          '実機ならカメラの前に赤い物を置いてください.'
                          .format(NO_BOX_HINT_S))
            if box is not None:
                hinted = True  # seen once: later losses are real tracking
            ri.go_velocity(0.0, 0.0, vtheta)
            ri.sleep(0.1)
    finally:
        ri.stop()
    print('stopped by {} after {:.1f} s'.format(reason, ri.now() - start))
    if args.save and rgb is not None:
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        if box is not None:
            cx, cy, w, h = box
            cv2.rectangle(bgr, (int(cx - w / 2), int(cy - h / 2)),
                          (int(cx + w / 2), int(cy + h / 2)), (0, 255, 0), 2)
        cv2.imwrite(args.save, bgr)
        print('saved', args.save)


if __name__ == '__main__':
    main()
