#!/usr/bin/env python3
"""Isaac Sim as Jedy's bus: the servicer side of actuo's shared memory.

    ros2 launch jedy_bringup jedy_isaac.launch.py     # starts this, with the rest

It takes the place actuo_bus_node takes on the real robot -- creates the same
segment, holds the same servicer lock, writes MEASURED state first and then
services commands at a fixed rate -- so ros2_control runs exactly as on the
robot: actuo_hardware's ActuoSystem, jedy_fullbody_controllers.yaml, the same
five trajectory controllers and the mecanum controller, all under
/jedy_fullbody. What is behind the segment is Isaac Sim instead of a board:

* the 18 position joints: commands go to Isaac on
  ``/jedy_fullbody/isaac/joint_command`` and Isaac's measured angles come back
  on ``/jedy_fullbody/isaac/joint_states``;
* the four wheels (``--base wheels``, the default; the USD from
  make_jedy_usd.py --base wheels, where the robot stands on its wheels with
  mecanum rollers on a floor): the wheel speeds mecanum_drive_controller
  commands go to Isaac as the wheels' velocity targets, and where the base
  goes is up to the rollers and the floor.

With ``--base planar`` (the older USD that mounts the base on planar joints
base_x_joint, base_y_joint, base_yaw_joint, and nothing touches the floor) the
wheel speeds are turned back into a body twist with the controller's own
kinematics (same wheel radius and centre projection, read from the controllers
yaml) and integrated into the planar joints' targets.

As on the robot, the wheels report no position and no speed (NaN), so nothing
downstream integrates a number this servicer made up.

Servo OFF is jedy_sim_servo's (the SetServo action) and Isaac's (the drives).
This servicer only follows what Isaac reports as released
(``/jedy_fullbody/servo/released_state``), as the real bus does with a servo
whose torque is off: it stops passing the controller's command on -- a
released joint is sent where it is, a released wheel a zero speed -- so
nothing is pushing the moment it is driven again.
"""

import argparse
import fcntl
import math
import os
import sys
import threading
import time

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String


# actuo is installed in this package's venv.
from actuo.drivers.shm import (  # noqa: E402
    CMD_STRIDE, STATE_PRESENT, STATE_STRIDE, ShmBridge, lock_path)

BASE_JOINTS = ["base_x_joint", "base_y_joint", "base_yaw_joint"]
#: mecanum_drive_controller's wheel order, by the yaml key naming each one.
WHEEL_KEYS = ["front_left_wheel_command_joint_name",
              "front_right_wheel_command_joint_name",
              "rear_left_wheel_command_joint_name",
              "rear_right_wheel_command_joint_name"]


def share_file(package, *parts):
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory(package), *parts)


def load_joint_order(config_path):
    """Joints in shm order: the bus config's, which is the URDF's."""
    with open(config_path) as f:
        return [j["name"] for j in yaml.safe_load(f)["joints"]]


def load_mecanum(controllers_path):
    """Wheel names in the controller's FL, FR, RL, RR order, radius, k."""
    with open(controllers_path) as f:
        doc = yaml.safe_load(f)
    params = doc["/jedy_fullbody"]["mecanum_drive_controller"]["ros__parameters"]
    kin = params["kinematics"]
    return ([params[k] for k in WHEEL_KEYS], float(kin["wheels_radius"]),
            float(kin["sum_of_robot_center_projection_on_X_Y_axis"]))


def body_twist(w, radius, k):
    """Forward kinematics of mecanum_drive_controller (inverse of its IK).

    Parameters
    ----------
    w : sequence of float
        Wheel speeds [rad/s] in FL, FR, RL, RR order.

    Returns
    -------
    tuple of float
        (vx, vy, wz) in the base frame [m/s, m/s, rad/s].
    """
    fl, fr, rl, rr = w
    vx = radius * (fl + fr + rl + rr) / 4.0
    vy = radius * (-fl + fr + rl - rr) / 4.0
    wz = radius * (-fl + fr - rl + rr) / (4.0 * k)
    return vx, vy, wz


def acquire_lock(shm_name):
    """The lock actuo_hardware checks to know a servicer is alive."""
    path = lock_path(shm_name)
    handle = open(path, "w")
    try:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        handle.close()
        raise RuntimeError(
            "another servicer already holds shm {!r} ({}). Stop it first: two "
            "servicers on one segment fight over every command.".format(
                shm_name, path))
    handle.write("{}\n".format(os.getpid()))
    handle.flush()
    return handle


class IsaacLink(Node):
    """Isaac's joint states in, joint commands out."""

    def __init__(self, ns, servo_ns):
        super().__init__("jedy_isaac_servicer")
        self._lock = threading.Lock()
        self.positions = {}
        self._released = frozenset()
        self.seen = threading.Event()
        self.pub = self.create_publisher(JointState, ns + "/joint_command", 10)
        self.create_subscription(JointState, ns + "/joint_states",
                                 self._on_state, qos_profile_sensor_data)
        self.create_subscription(String, servo_ns + "/released_state",
                                 self._on_released, 10)

    def _on_released(self, msg):
        with self._lock:
            self._released = frozenset(
                n for n in msg.data.split(",") if n and not n.startswith("!"))

    def released(self):
        """The joints Isaac has released (its last report)."""
        with self._lock:
            return self._released

    def _on_state(self, msg):
        with self._lock:
            self.positions.update(zip(msg.name, msg.position))
        self.seen.set()

    def measured(self):
        with self._lock:
            return dict(self.positions)

    def command(self, names, positions, velocities=None):
        """One command for every joint named.

        Isaac's articulation controller takes position and velocity targets
        from the same message, so with ``velocities`` each joint gets both: a
        position-driven joint ignores a zero velocity target the way it would
        anyway, and a velocity-driven wheel (no stiffness) ignores the
        position.
        """
        if not names:
            # Everything released: nothing to command. An empty message is
            # not "command nothing" to Isaac's articulation controller.
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(names)
        msg.position = [float(p) for p in positions]
        if velocities is not None:
            msg.velocity = [float(v) for v in velocities]
        self.pub.publish(msg)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--shm-name", default="jedy_fullbody")
    p.add_argument("--config", default=share_file(
        "jedy_bringup", "config", "jedy_fullbody_rcb4.yaml"),
        help="bus config whose joint order is the shm layout")
    p.add_argument("--controllers", default=share_file(
        "jedy_bringup", "config", "jedy_fullbody_controllers.yaml"))
    p.add_argument("--wheels", action="store_true",
                   help="the URDF was expanded with wheels:=true")
    p.add_argument("--base", choices=["planar", "wheels"], default="wheels",
                   help="the USD's base (make_jedy_usd.py --base): planar "
                        "joints driven from the wheel speeds, or wheels on "
                        "the floor")
    p.add_argument("--isaac-ns", default="/jedy_fullbody/isaac")
    p.add_argument("--servo-ns", default="/jedy_fullbody/servo")
    p.add_argument("--rate", type=float, default=100.0)
    p.add_argument("--timeout", type=float, default=300.0,
                   help="seconds to wait for Isaac's first joint state")
    # A launch Node appends --ros-args (use_sim_time); those are rclpy's.
    args = p.parse_args(rclpy.utilities.remove_ros_args(
        sys.argv if argv is None else argv)[1:])

    joints = load_joint_order(args.config)
    wheel_names, radius, k = load_mecanum(args.controllers)
    if not args.wheels:
        joints = [j for j in joints if j not in wheel_names]
    is_wheel = np.array([j in wheel_names for j in joints])
    position_joints = [j for j, w in zip(joints, is_wheel) if not w]
    wheel_index = [joints.index(w) for w in wheel_names] if args.wheels else []
    n = len(joints)

    rclpy.init(args=sys.argv if argv is None else argv)
    link = IsaacLink(args.isaac_ns.rstrip("/"), args.servo_ns.rstrip("/"))
    spinner = threading.Thread(target=rclpy.spin, args=(link,), daemon=True)
    spinner.start()

    print("[jedy_isaac_servicer] waiting for Isaac on {}/joint_states".format(
        args.isaac_ns), flush=True)
    if not link.seen.wait(args.timeout):
        raise TimeoutError("no joint state from Isaac within {} s".format(
            args.timeout))
    measured = link.measured()
    on_wheels = args.wheels and args.base == "wheels"
    required = position_joints + (wheel_names if on_wheels else BASE_JOINTS)
    missing = [j for j in required if j not in measured]
    if missing:
        raise RuntimeError(
            "Isaac does not report {}: the USD does not match this robot's "
            "joints".format(missing))

    _lock = acquire_lock(args.shm_name)  # noqa: F841 -- held for life
    shm = ShmBridge(args.shm_name, n_joints=n, cmd_stride=CMD_STRIDE,
                    state_stride=STATE_STRIDE, create=True)

    def write_state(measured):
        state = np.full((n, STATE_STRIDE), math.nan)
        for i, name in enumerate(joints):
            if not is_wheel[i]:
                state[i, 0] = measured[name]
                state[i, 1] = math.nan     # as on the robot: not sensed
        state[:, STATE_PRESENT] = 1.0
        shm.write_state(state)

    # MEASURED first: actuo_hardware seeds its command from this.
    write_state(measured)
    base = (None if on_wheels
            else np.array([measured[j] for j in BASE_JOINTS], dtype=float))
    print("[jedy_isaac_servicer] created shm {!r} ({} joints{}); servicing "
          "at {} Hz".format(args.shm_name, n,
                            "" if not args.wheels
                            else ", wheels on the floor" if on_wheels
                            else ", wheels -> planar base",
                            args.rate), flush=True)

    clock = link.get_clock()
    last = clock.now().nanoseconds
    period = 1.0 / args.rate
    cycles = 0
    try:
        while rclpy.ok():
            started = time.monotonic()
            command = np.asarray(shm.read_command(), dtype=float).reshape(
                n, CMD_STRIDE)
            measured = link.measured()
            released = link.released()
            names, targets = [], []
            for i, name in enumerate(joints):
                if is_wheel[i]:
                    continue
                target = command[i, 0]
                # Released (servo OFF): the joint is not commanded at all,
                # as a real servo ignores commands with its torque off.
                # Commanding it even to where it is now leaks force past
                # the released drive: a fixed-base Jedy whose rarm_joint0
                # held still uncommanded crept 0.26 deg/s when it was sent
                # its own position every tick (measured 2026-11-07).
                if name in released:
                    continue
                # NaN: no controller owns the slot -- hold where it is.
                names.append(name)
                targets.append(measured[name] if math.isnan(target)
                               else target)
            now = clock.now().nanoseconds
            dt = max(0.0, (now - last) * 1e-9)
            last = now
            velocities = None
            if on_wheels:
                driven = [(i, j) for i, j in zip(wheel_index, wheel_names)
                          if j not in released]
                w = [0.0 if math.isnan(command[i, 1]) else command[i, 1]
                     for i, _ in driven]
                velocities = [0.0] * len(names) + w
                names += [j for _, j in driven]
                targets += [measured[j] for _, j in driven]
            elif args.wheels:
                w = [0.0 if math.isnan(command[i, 1]) else command[i, 1]
                     for i in wheel_index]
                vx, vy, wz = body_twist(w, radius, k)
                yaw = base[2]
                base[0] += (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
                base[1] += (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
                base[2] += wz * dt
                names += BASE_JOINTS
                targets += list(base)
            link.command(names, targets, velocities)
            write_state(measured)
            cycles += 1
            if cycles % 1000 == 0 and base is not None:
                print("[jedy_isaac_servicer] {} cycles, base x={:.3f} y={:.3f} "
                      "yaw={:.3f}".format(cycles, *base), flush=True)
            time.sleep(max(0.0, period - (time.monotonic() - started)))
    except KeyboardInterrupt:
        pass
    print("[jedy_isaac_servicer] {} cycles".format(cycles), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
