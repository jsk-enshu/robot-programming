#!/usr/bin/env python3
"""Say which axis or button just moved, so a pad's numbering is read not guessed.

    ros2 run jedy_bringup jedy_joy_buttons

Press one thing at a time. The indices move between kernel drivers and pad
models, and a wrong one in jedy_joy.py is not an error -- it is a button that
silently does something else, which on a robot that can drive away is the worst
kind of wrong.
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import Joy  # noqa: E402

from jedy_teleop.mapping import AXIS_ROLES, BUTTON_ROLES  # noqa: E402

#: What jedy_joy.py assumes, read from the same jedy_teleop.mapping it uses,
#: so this list cannot drift from the node. Printed beside whatever is pressed.
ASSUMED = {}
ASSUMED.update({"axis {}".format(i): "{} ({})".format(*r)
                for i, r in AXIS_ROLES.items()})
ASSUMED.update({"button {}".format(i): "{} -- {}".format(*r)
                for i, r in BUTTON_ROLES.items()})


def main(argv=None):
    rclpy.init(args=argv)
    node = Node("jedy_joy_buttons")
    last = {"msg": None}

    def on_joy(msg):
        previous = last["msg"]
        last["msg"] = msg
        if previous is None:
            node.get_logger().info(
                "{} axes, {} buttons. Press one thing at a time.".format(
                    len(msg.axes), len(msg.buttons)))
            return
        for i, (now, before) in enumerate(zip(msg.axes, previous.axes)):
            if abs(now - before) > 0.4:
                key = "axis {}".format(i)
                print("{:<10} = {:+.2f}   {}".format(
                    key, now, ASSUMED.get(key, "(not used by jedy_joy)")),
                    flush=True)
        for i, (now, before) in enumerate(zip(msg.buttons, previous.buttons)):
            if now and not before:
                key = "button {}".format(i)
                print("{:<10} pressed    {}".format(
                    key, ASSUMED.get(key, "(not used by jedy_joy)")), flush=True)

    node.create_subscription(Joy, "/joy", on_joy, 10)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
