#!/usr/bin/env python3
"""Check the pubsub_sample firmware in both directions.

Counts /uno_r4/counter for a while, then publishes to /uno_r4/led. The firmware
copies the received value into its counter, so seeing the counter jump to that
value proves that PC -> board works as well as board -> PC.

Run with micro_ros_agent already connected::

    python3 scripts/check_pubsub.py
"""
import time

import rclpy
from std_msgs.msg import Int32

LISTEN_SEC = 10.0
SENT_VALUE = 1000
SEND_TIMES = 5
SEND_INTERVAL_SEC = 0.3
AFTER_SEC = 2.0


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
        rclpy.spin_once(node, timeout_sec=0.05)


def main():
    """Report the counter rate and whether the board echoes a sent value."""
    rclpy.init()
    node = rclpy.create_node("uno_r4_pubsub_checker")
    got = []
    node.create_subscription(Int32, "uno_r4/counter", lambda m: got.append(m.data), 10)
    pub = node.create_publisher(Int32, "uno_r4/led", 10)

    spin_for(node, LISTEN_SEC)
    print(f"received {len(got)} msgs in {LISTEN_SEC:.0f} s, last values: {got[-5:]}")

    n = len(got)
    for _ in range(SEND_TIMES):
        pub.publish(Int32(data=SENT_VALUE))
        spin_for(node, SEND_INTERVAL_SEC)
    spin_for(node, AFTER_SEC)
    echoed = [v for v in got[n:] if v >= SENT_VALUE]
    print(f"after publishing {SENT_VALUE} to /uno_r4/led: {got[n:n + 8]}")
    print("PC -> board: OK" if echoed else "PC -> board: NOT confirmed")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
