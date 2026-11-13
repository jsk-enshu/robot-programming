#!/usr/bin/env python3
"""Say which axis or button just moved, so a pad's numbering is read not guessed.

    ros2 run jedy_bringup jedy_joy_buttons

Press one thing at a time. The indices move between kernel drivers and pad
models, and a wrong one in jedy_joy.py is not an error -- it is a button that
silently does something else, which on a robot that can drive away is the worst
kind of wrong.
"""

import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

#: What jedy_joy.py currently assumes. Printed beside whatever is pressed so
#: the two can be compared without opening the file.
ASSUMED = {
    "axis 0": "left stick X (strafe)",
    "axis 1": "left stick Y (forward)",
    "axis 3": "right stick X (turn)",
    "axis 6": "D-pad X (head pan)",
    "axis 7": "D-pad Y (head tilt)",
    "button 1": "CIRCLE -- release torque",
    "button 2": "TRIANGLE -- torque on",
    "button 4": "L1 -- deadman",
    "button 5": "R1 -- slow",
}


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
