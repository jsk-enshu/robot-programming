#!/usr/bin/env python3
"""Every axis and button, live, next to what the teleop node thinks it is.

    ros2 run jedy_bringup jedy_joy_monitor

Press a button or move a stick and watch its row change. The label beside it is
read from jedy_teleop.mapping -- the same module the teleop node imports -- so
what you see here IS what the robot will act on. If the row that moves is not
the row you expected, the mapping is wrong and this is where you find out,
rather than after the robot has driven somewhere.

The bottom line shows what the node would command RIGHT NOW, deadman included,
so "is L1 the deadman on this pad" is answerable by holding it and looking.

Nothing is published. This only watches.
"""

from __future__ import annotations

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import Joy  # noqa: E402

from jedy_teleop.deadband import scale_to_usable  # noqa: E402
from jedy_teleop.mapping import (  # noqa: E402
    AXIS_LEFT_X, AXIS_LEFT_Y, AXIS_RIGHT_X, AXIS_ROLES, BUTTON_L1,
    BUTTON_R1, BUTTON_ROLES)

CLEAR = "\033[2J\033[H"
BOLD, DIM, GREEN, YELLOW, RED, RESET = (
    "\033[1m", "\033[2m", "\033[32m", "\033[33m", "\033[31m", "\033[0m")


def bar(value, width=21):
    """A centred bar, so a stick at rest is visibly at rest."""
    middle = width // 2
    cells = ["-"] * width
    cells[middle] = "|"
    position = int(round(middle + value * middle))
    position = max(0, min(width - 1, position))
    cells[position] = "#"
    return "".join(cells)


def parse_args(argv):
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    # jedy_fullbody's base, as jedy_joy defaults to.
    p.add_argument("--wheel-radius", type=float, default=0.02525)
    p.add_argument("--centre-projection", type=float, default=0.094)
    p.add_argument("--min-wheel-rad-s", type=float, default=2.83)
    p.add_argument("--max-wheel-rad-s", type=float, default=6.85)
    p.add_argument("--ros-args", dest="ros_args", nargs=argparse.REMAINDER,
                   default=[])
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    rclpy.init(args=args.ros_args or None)
    node = Node("jedy_joy_monitor")
    state = {"msg": None, "stamp": None, "count": 0, "moved": set(),
             "pressed": set()}

    def on_joy(msg):
        previous = state["msg"]
        now = time.time()
        if previous is not None:
            for i, (a, b) in enumerate(zip(msg.axes, previous.axes)):
                if abs(a - b) > 0.2:
                    state["moved"].add(i)
            for i, (a, b) in enumerate(zip(msg.buttons, previous.buttons)):
                if a and not b:
                    state["pressed"].add(i)
        state["msg"] = msg
        state["stamp"] = now
        state["count"] += 1

    node.create_subscription(Joy, "/joy", on_joy, 10)

    def draw():
        msg = state["msg"]
        out = [CLEAR, BOLD + "jedy_joy_monitor" + RESET +
               "   press something and watch its row.  Ctrl-C to stop\n"]
        if msg is None:
            out.append(RED + "no /joy messages yet.\n" + RESET)
            out.append(DIM + "  is joy_linux_node running?  "
                             "ros2 launch jedy_bringup jedy_joy.launch.py\n" + RESET)
            print("".join(out), flush=True)
            return

        age = time.time() - state["stamp"]
        health = (GREEN + "live" if age < 0.3 else RED + "STALE {:.1f}s".format(age))
        out.append("  /joy {} {}   {} messages   {} axes  {} buttons\n\n".format(
            health, RESET, state["count"], len(msg.axes), len(msg.buttons)))

        out.append(BOLD + "  AXES\n" + RESET)
        for i, value in enumerate(msg.axes):
            name, role = AXIS_ROLES.get(i, ("", ""))
            used = bool(name)
            seen = "*" if i in state["moved"] else " "
            colour = (GREEN if abs(value) > 0.15 and used else
                      YELLOW if abs(value) > 0.15 else DIM if not used else "")
            out.append("  {}{}axis {:<2d} [{}] {:+.2f}  {:<14s} {}{}\n".format(
                colour, seen, i, bar(value), value,
                name or "(unused)", role, RESET))

        out.append("\n" + BOLD + "  BUTTONS\n" + RESET)
        for i, value in enumerate(msg.buttons):
            name, role = BUTTON_ROLES.get(i, ("", ""))
            used = bool(name)
            seen = "*" if i in state["pressed"] else " "
            colour = (GREEN if value and used else YELLOW if value else
                      DIM if not used else "")
            out.append("  {}{}button {:<2d} {:<4s}  {:<10s} {}{}\n".format(
                colour, seen, i, "ON" if value else "..",
                name or "(unused)", role, RESET))

        # What the node would do with this, right now. The point of the whole
        # tool: "is L1 the deadman on this pad" is answered by holding it.
        held = len(msg.buttons) > BUTTON_L1 and msg.buttons[BUTTON_L1]
        out.append("\n" + BOLD + "  WHAT THE ROBOT WOULD DO\n" + RESET)
        if not held:
            out.append("  " + DIM + "deadman not held -- the base stays "
                       "stopped\n" + RESET)
        else:
            slow = (0.55 if len(msg.buttons) > BUTTON_R1 and msg.buttons[BUTTON_R1]
                    else 1.0)
            top = args.max_wheel_rad_s * args.wheel_radius
            vx = msg.axes[AXIS_LEFT_Y] * top * slow
            vy = msg.axes[AXIS_LEFT_X] * top * slow
            wz = msg.axes[AXIS_RIGHT_X] * top / args.centre_projection * slow
            vx, vy, wz, note = scale_to_usable(
                vx, vy, wz, args.wheel_radius, args.centre_projection,
                args.min_wheel_rad_s, args.max_wheel_rad_s)
            out.append("  " + GREEN + "DEADMAN HELD" + RESET +
                       "   vx {:+.3f}  vy {:+.3f}  wz {:+.3f} m/s, rad/s\n".format(
                           vx, vy, wz))
            if note:
                out.append("  " + YELLOW + "  " + note + RESET + "\n")
        out.append("\n" + DIM +
                   "  * = this one has moved at least once since start\n" + RESET)
        print("".join(out), flush=True)

    node.create_timer(0.1, draw)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(RESET)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
