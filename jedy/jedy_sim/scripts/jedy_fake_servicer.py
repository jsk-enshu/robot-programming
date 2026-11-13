#!/usr/bin/env python3
"""A servicer that drives NOTHING, so the collision gate can be exercised.

    ros2 run jedy_sim jedy_fake_servicer                                # here
    ros2 launch jedy_bringup jedy_fullbody.launch.py use_sim:=true      # there

The launch must run with its default wheels:=false: this services the 18
position joints only.

It occupies exactly the position the real one does -- attaches the same shared
memory segment, reads the same stride-3 command block, writes the same stride-2
state block -- and between the two it runs the real
``actuo.safety.CollisionGate``. What it does NOT have is a robot: the plant is an
ideal servo that reports back whatever the gate allowed. So this exercises the
PLUMBING (controller -> ros2_control -> shm -> gate -> state -> /joint_states)
without hardware and without physics.

Two things it deliberately is not:

* **Not a simulator.** There is no dynamics, no gravity, no tracking error. A
  joint reaches its command in one cycle. Anything you conclude here about
  timing or overshoot is meaningless; anything you conclude about whether the
  gate stops a trajectory is real.
* **Not safe to confuse with the real one.** It prints what it is on every
  start, and its shm name defaults to the same segment precisely so that
  running both at once fails loudly rather than silently splitting the robot's
  commands between two writers.
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np


def _add_pkg_path(package: str) -> None:
    from ament_index_python.packages import get_package_share_directory
    share = get_package_share_directory(package)
    for candidate in (share, os.path.join(share, "scripts")):
        if candidate not in sys.path:
            sys.path.insert(0, candidate)


# actuo (shared memory, collision gate) is installed in this package's venv.
from actuo.drivers.shm import ShmBridge  # noqa: E402

#: Must match actuo_hardware's kCmdStride / the state block the real servicer
#: writes. Duplicated from actuo_bus_node deliberately -- importing that script
#: would drag in the whole bus stack for two integers.
CMD_STRIDE = 3
STATE_STRIDE = 2

#: The 18 position joints, in the controller's order. Same list, same order,
#: as jedy_fullbody_rcb4.yaml and the <ros2_control> block of
#: jedy_fullbody_actuo.urdf.xacro expanded with wheels:=false: that order IS
#: the shm layout. (With wheels:=true the four wheel joints follow these.)
JOINTS = ["head_joint0", "head_joint1",
          "rarm_joint0", "rarm_joint1", "rarm_joint2", "rarm_joint3",
          "rarm_joint4", "rarm_joint5", "rarm_joint6",
          "larm_joint0", "larm_joint1", "larm_joint2", "larm_joint3",
          "larm_joint4", "larm_joint5", "larm_joint6",
          "rarm_gripper_joint", "larm_gripper_joint"]


def build_filter(decelerate):
    """The SAME filter the real servicer is given by --command-filter.

    Not a local copy of the gate configuration. This process exists to rehearse
    the real one, and a rehearsal that configures safety differently from the
    performance rehearses nothing -- it was a local CollisionGate for exactly
    one commit, and that was already one source of truth too many.
    """
    _add_pkg_path("jedy_safety")
    from jedy_safety.collision_filter import JedyCollisionFilter
    return JedyCollisionFilter(JOINTS, decelerate=decelerate)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--shm-name", default="jedy_fullbody")
    parser.add_argument("--rate", type=float, default=100.0)
    parser.add_argument("--timeout", type=float, default=90.0,
                        help="how long to wait for the C++ side to create the "
                             "segment")
    parser.add_argument("--decelerate", action="store_true",
                        help="distance-based slow-down instead of a hard stop")
    parser.add_argument("--no-gate", action="store_true",
                        help="pass commands straight through. Use it once, to "
                             "see what the gate is preventing.")
    args = parser.parse_args(argv)

    print("[jedy_fake_servicer] THIS DRIVES NOTHING. Ideal servo, no physics; "
          "the gate in the path is the real one.", flush=True)

    command_filter = None
    if not args.no_gate:
        command_filter = build_filter(args.decelerate)
    else:
        print("[jedy_fake_servicer] gate DISABLED (--no-gate)", flush=True)

    n = len(JOINTS)
    deadline = time.time() + args.timeout
    shm = None
    while shm is None:
        try:
            shm = ShmBridge(args.shm_name, n_joints=n, cmd_stride=CMD_STRIDE,
                            state_stride=STATE_STRIDE, create=False)
        except (FileNotFoundError, ValueError) as error:
            if time.time() > deadline:
                raise TimeoutError(
                    "shm {!r} never appeared ({}). Start the launch: it is the "
                    "C++ side that creates the segment.".format(
                        args.shm_name, error))
            time.sleep(0.1)

    # The handshake the real servicer follows: write MEASURED state first, so
    # on_activate can seed its command from it rather than from the URDF. Here
    # "measured" is the zero pose, which is this plant's whole truth.
    applied = np.zeros(n)
    state = np.zeros((n, STATE_STRIDE), dtype=np.float64)
    state[:, 0] = applied
    shm.write_state(state)
    print("[jedy_fake_servicer] attached shm {!r} ({} joints); servicing at "
          "{} Hz".format(args.shm_name, n, args.rate), flush=True)

    dt = 1.0 / args.rate
    next_t = time.time()
    cycles = 0
    gated_cycles = 0
    was_stopped = False
    try:
        while True:
            command = np.asarray(shm.read_command(), dtype=float)[0::CMD_STRIDE][:n]
            if command_filter is None:
                applied = command
            else:
                result = command_filter.filter(command)
                applied = np.asarray(result.applied, dtype=float)
                if result.gated:
                    gated_cycles += 1
                    if not was_stopped:
                        print("[jedy_fake_servicer] GATE STOPPED at cycle {}: "
                              "{}".format(cycles, result.reason), flush=True)
                        print("           commanded {}".format(
                            np.round(np.rad2deg(command), 1)), flush=True)
                        print("           holding   {}".format(
                            np.round(np.rad2deg(applied), 1)), flush=True)
                    was_stopped = True
                elif was_stopped:
                    print("[jedy_fake_servicer] gate released", flush=True)
                    was_stopped = False

            # Ideal servo: the joint is wherever the gate let it be commanded.
            state[:, 0] = applied
            state[:, 1] = 0.0
            shm.write_state(state)

            cycles += 1
            if cycles % 500 == 0:
                print("[jedy_fake_servicer] {} cycles, {} gated".format(
                    cycles, gated_cycles), flush=True)
            next_t += dt
            sleep = next_t - time.time()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.time()
    except KeyboardInterrupt:
        print("\n[jedy_fake_servicer] {} cycles, {} gated".format(
            cycles, gated_cycles), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
