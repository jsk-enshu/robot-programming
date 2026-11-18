#!/usr/bin/env python3
"""Check the mechatrobot firmware: topic rates, the Range message, and the servo.

Run with micro_ros_agent already connected::

    python3 scripts/check_mechatrobot.py            # moves the servo to 30 deg
    python3 scripts/check_mechatrobot.py --target 0
"""
import argparse
import time

import rclpy
from sensor_msgs.msg import Range
from std_msgs.msg import Bool
from std_msgs.msg import Int64

WARMUP_SEC = 3.0
MEASURE_SEC = 5.0
TRACK_SEC = 8.0
TRACE_INTERVAL_SEC = 0.5


def spin_for(node, sec):
    """Spin ``node`` for ``sec`` seconds.

    Parameters
    ----------
    node : rclpy.node.Node
        Node to spin.
    sec : float
        Duration in seconds.
    """
    t0 = time.time()
    while time.time() - t0 < sec:
        rclpy.spin_once(node, timeout_sec=0.02)


def main():
    """Print topic rates, the last Range message and the servo trajectory."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", type=int, default=30, help="servo target [deg]")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("mechatrobot_checker")
    ranges, leds, positions = [], [], []
    node.create_subscription(Range, "range", ranges.append, 10)
    node.create_subscription(Bool, "led/state", lambda m: leds.append(m.data), 10)
    node.create_subscription(Int64, "motor1/position", lambda m: positions.append(m.data), 10)
    cmd = node.create_publisher(Int64, "motor1/command", 10)

    spin_for(node, WARMUP_SEC)
    ranges.clear()
    leds.clear()
    positions.clear()
    spin_for(node, MEASURE_SEC)
    print(f"range           {len(ranges) / MEASURE_SEC:5.1f} Hz (expected 20)")
    print(f"led/state       {len(leds) / MEASURE_SEC:5.1f} Hz (expected 2) {leds[:4]}")
    print(f"motor1/position {len(positions) / MEASURE_SEC:5.1f} Hz (expected 20)")
    if ranges:
        r = ranges[-1]
        now = node.get_clock().now().nanoseconds * 1e-9
        stamp = r.header.stamp.sec + r.header.stamp.nanosec * 1e-9
        print(f"range: frame_id={r.header.frame_id!r} min={r.min_range:.2f} "
              f"max={r.max_range:.2f} range={r.range:.3f} m "
              f"(stamp - now = {stamp - now:+.3f} s)")

    for _ in range(3):
        cmd.publish(Int64(data=args.target))
        spin_for(node, 0.2)
    trace = []
    t0 = time.time()
    while time.time() - t0 < TRACK_SEC:
        spin_for(node, TRACE_INTERVAL_SEC)
        if positions:
            trace.append(positions[-1])
    print(f"motor1/command {args.target} deg -> motor1/position every "
          f"{TRACE_INTERVAL_SEC} s: {trace}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
