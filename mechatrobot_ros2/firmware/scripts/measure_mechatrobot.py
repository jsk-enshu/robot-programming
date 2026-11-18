#!/usr/bin/env python3
"""Measure the mechatrobot's ROS 2 interface, for comparing firmware versions.

Works with either the micro-ROS firmware (with micro_ros_agent) or the serial
firmware (with serial_bridge.py), since both expose the same topics. Start the
agent or the bridge first, then::

    python3 measure_mechatrobot.py --label microros
    python3 measure_mechatrobot.py --label serial

Reports
-------
- time until the first ``range`` message arrives
- ``range`` rate and the spread of its arrival intervals
- ``range.header.stamp`` minus the time it arrived on this PC
- command latency: time from publishing ``motor1/command`` until
  ``motor1/position`` reports a different value
"""
import argparse
import statistics
import time

import rclpy
from sensor_msgs.msg import Range
from std_msgs.msg import Int64

FIRST_MSG_TIMEOUT_SEC = 15.0
COLLECT_SEC = 20.0
TRIALS = 20
STEP_DEG = 3
TRIAL_TIMEOUT_SEC = 1.0
SETTLE_SEC = 0.5


class Recorder:
    """Keep arrival times and values of the measured topics."""

    def __init__(self, node):
        self.node = node
        self.range_arrivals = []
        self.range_stamp_offsets = []
        self.position = None
        self.position_changed_at = None
        node.create_subscription(Range, "range", self.on_range, 50)
        node.create_subscription(Int64, "motor1/position", self.on_position, 50)
        self.command_pub = node.create_publisher(Int64, "motor1/command", 10)

    def on_range(self, msg):
        """Record arrival time and stamp offset of a Range message.

        Parameters
        ----------
        msg : sensor_msgs.msg.Range
            Received message.
        """
        now = self.node.get_clock().now().nanoseconds * 1e-9
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.range_arrivals.append(time.monotonic())
        self.range_stamp_offsets.append(stamp - now)

    def on_position(self, msg):
        """Record the latest position and when it last changed.

        Parameters
        ----------
        msg : std_msgs.msg.Int64
            Reported motor angle in degrees.
        """
        if self.position is not None and msg.data != self.position:
            self.position_changed_at = time.monotonic()
        self.position = msg.data


def spin_until(node, predicate, timeout_sec):
    """Spin until ``predicate()`` is true or the timeout expires.

    Parameters
    ----------
    node : rclpy.node.Node
        Node to spin.
    predicate : callable
        Condition checked after every spin.
    timeout_sec : float
        Maximum time to spin.

    Returns
    -------
    bool
        True if the condition became true before the timeout.
    """
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_sec:
        rclpy.spin_once(node, timeout_sec=0.005)
        if predicate():
            return True
    return False


def ms(values):
    """Format a list of seconds as 'median / max' in milliseconds.

    Parameters
    ----------
    values : list of float
        Durations in seconds.

    Returns
    -------
    str
        Human-readable summary.
    """
    if not values:
        return "n/a"
    return (f"median {statistics.median(values) * 1e3:6.1f} ms, "
            f"min {min(values) * 1e3:6.1f}, max {max(values) * 1e3:6.1f} (n={len(values)})")


def main():
    """Run all measurements and print a summary."""
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--label", default="", help="name printed with the results")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("mechatrobot_measure")
    rec = Recorder(node)

    t0 = time.monotonic()
    if not spin_until(node, lambda: len(rec.range_arrivals) > 0, FIRST_MSG_TIMEOUT_SEC):
        print(f"[{args.label}] no range message within {FIRST_MSG_TIMEOUT_SEC} s")
        rclpy.shutdown()
        return
    first_msg_sec = time.monotonic() - t0

    rec.range_arrivals.clear()
    rec.range_stamp_offsets.clear()
    spin_until(node, lambda: False, COLLECT_SEC)
    arrivals = rec.range_arrivals[:]
    gaps = [b - a for a, b in zip(arrivals, arrivals[1:])]
    offsets = rec.range_stamp_offsets[:]

    latencies = []
    misses = 0
    for i in range(TRIALS):
        target = (rec.position or 0) + (STEP_DEG if i % 2 == 0 else -STEP_DEG)
        rec.position_changed_at = None
        sent = time.monotonic()
        rec.command_pub.publish(Int64(data=target))
        if spin_until(node, lambda: rec.position_changed_at is not None, TRIAL_TIMEOUT_SEC):
            latencies.append(rec.position_changed_at - sent)
        else:
            misses += 1
        spin_until(node, lambda: rec.position == target, SETTLE_SEC)

    print(f"[{args.label}] first range message after {first_msg_sec:.2f} s")
    print(f"[{args.label}] range rate {len(arrivals) / COLLECT_SEC:.2f} Hz over {COLLECT_SEC:.0f} s")
    print(f"[{args.label}] range interval: {ms(gaps)}, stdev "
          f"{statistics.pstdev(gaps) * 1e3 if gaps else float('nan'):.1f} ms")
    print(f"[{args.label}] range stamp - arrival: {ms(offsets)}")
    print(f"[{args.label}] command -> position change: {ms(latencies)}, missed {misses}/{TRIALS}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
