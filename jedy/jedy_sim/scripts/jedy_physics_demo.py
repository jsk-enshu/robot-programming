#!/usr/bin/env python3
"""Unified demo: converted Jedy as a MuJoCo physics plant, started at its
reset pose, with actuo's CollisionGate stopping self-collisions -- optionally
in the interactive MuJoCo viewer.

    # headless (verify the gate stops the arm in physics)
    python jedy_physics_demo.py
    # interactive Gazebo-like window (macOS: run under mjpython)
    mjpython jedy_physics_demo.py --view

Pipeline shape: a commanded trajectory -> CollisionGate(skrobot self-collision,
reset-pose reference) -> MuJoCo position actuators -> physics. The arm is driven
toward the body; the gate freezes it at the last self-collision-free pose while
the rest of the robot holds its reset pose.
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

from actuo.safety import CollisionGate, SelfCollisionMonitor  # noqa: E402


def build_plant(mjcf_path):
    import mujoco
    model = mujoco.MjModel.from_xml_path(mjcf_path)
    data = mujoco.MjData(model)
    return mujoco, model, data


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--view", action="store_true", help="interactive viewer (mjpython)")
    p.add_argument("--joint", default="rarm_joint1", help="joint to drive into collision")
    p.add_argument("--target", type=float, default=1.2, help="raw target [rad]")
    p.add_argument("--mjcf", default="/tmp/jedy_mjcf/jedy.xml")
    args = p.parse_args()

    from skrobot.models import Jedy
    from skrobot.urdf import urdf_to_mjcf

    # 1. skrobot model at reset pose (the non-colliding home; init/zero collides)
    robot = Jedy()
    robot.reset_pose()
    reset_av = np.array(robot.angle_vector(), dtype=float)
    joint_names = [j.name for j in robot.joint_list]

    # 2. convert URDF -> MJCF (once)
    if not os.path.exists(args.mjcf):
        urdf_to_mjcf(robot.urdf_path, args.mjcf)
    mujoco, model, data = build_plant(args.mjcf)

    # 3. name-based map skrobot angle-vector index -> mujoco qpos/dof/actuator
    def jid(name):
        return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)

    def aid(name):
        return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name + "_act")

    qadr, dadr, actid = {}, {}, {}
    for name in joint_names:
        j = jid(name)
        if j < 0:
            continue
        qadr[name] = model.jnt_qposadr[j]
        dadr[name] = model.jnt_dofadr[j]
        actid[name] = aid(name)

    # 4. initialise the plant AT the reset pose (qpos and actuator targets)
    for name, i in zip(joint_names, range(len(joint_names))):
        if name in qadr:
            data.qpos[qadr[name]] = reset_av[i]
            if actid[name] >= 0:
                data.ctrl[actid[name]] = reset_av[i]
    mujoco.mj_forward(model, data)

    # 5. collision gate: reset pose is the reference (its collisions are the
    #    structural/adjacent ones to ignore); drive one joint from its reset value.
    monitor = SelfCollisionMonitor(robot, reference_angle_vector=reset_av)
    print("[demo] ignoring %d structural pairs at reset pose" % len(monitor.ignored_pairs))
    reset_val = reset_av[joint_names.index(args.joint)]
    gate = CollisionGate(monitor, [args.joint], base_angle_vector=reset_av,
                         initial_safe=[reset_val])

    viewer = None
    if args.view:
        import mujoco.viewer
        viewer = mujoco.viewer.launch_passive(model, data)

    # 6. drive the joint from its reset value toward `target`, gated.
    traj = np.linspace(reset_val, args.target, 200)
    ai = actid[args.joint]
    stop_reported = False
    t0 = time.time()
    for k, cmd in enumerate(traj):
        res = gate.filter([cmd])
        if ai >= 0:
            data.ctrl[ai] = res.applied[0]
        for _ in range(10):
            mujoco.mj_step(model, data)
        if res.stopped and not stop_reported:
            print("[demo] GATE STOP at cmd=%.3f -> hold %.3f (would hit %s)"
                  % (cmd, res.applied[0], res.colliding_pairs[0]))
            stop_reported = True
        if viewer is not None:
            if not viewer.is_running():
                break
            viewer.sync()
            time.sleep(0.01)

    achieved = data.qpos[qadr[args.joint]]
    print("[demo] raw target=%.3f  gate held=%.3f  plant achieved=%.3f"
          % (args.target, gate._last_safe[0], achieved))
    # verify the achieved plant pose is self-collision-free (via the monitor)
    av = reset_av.copy()
    for name in joint_names:
        if name in qadr:
            av[joint_names.index(name)] = data.qpos[qadr[name]]
    hit, pairs = monitor.check_angle_vector(av)
    print("[demo] plant pose self-collides: %s %s" % (hit, pairs[:2]))
    print("[demo] RESULT:", "PASS" if (stop_reported and not hit) else "CHECK")

    if viewer is not None:
        print("[demo] leaving viewer open ~20s")
        end = time.time() + 20
        while viewer.is_running() and time.time() < end:
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(0.01)
        viewer.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
