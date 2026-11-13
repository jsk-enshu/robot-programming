#!/usr/bin/env python3
"""Jedy MuJoCo bus-servicer for the actuo pipeline.

Attaches to the shared-memory segment the C++ ActuoSystem creates for the /jedy
controllers, drives a converted-Jedy MuJoCo physics plant with the 14 arm joints
(others held at reset pose), passes every command through the CollisionGate
(self-collision, reset-pose reference), and writes joint state back so
/jedy/joint_states reflects the physics.

    # headless
    python jedy_servicer.py
    # with the interactive physics viewer (macOS: mjpython)
    mjpython jedy_servicer.py --view

Run alongside:  ros2 launch jedy_bringup jedy.launch.py
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np


# actuo (shared memory, collision gate) is installed in this package's venv.
from actuo.drivers.shm import ShmBridge  # noqa: E402
from actuo.safety import CollisionGate, SelfCollisionMonitor  # noqa: E402

ARM = ["rarm_joint%d" % i for i in range(7)] + \
      ["larm_joint%d" % i for i in range(7)]


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--shm-name", default="jedy")
    p.add_argument("--rate", type=float, default=200.0)
    p.add_argument("--timeout", type=float, default=60.0)
    p.add_argument("--view", action="store_true")
    p.add_argument("--mode", default="position",
                   choices=["position", "velocity", "effort"],
                   help="operating mode (must match the launched controller)")
    p.add_argument("--mjcf", default="")
    p.add_argument("--no-gate", action="store_true", help="disable collision gate")
    p.add_argument("--decelerate", action="store_true",
                   help="distance-based deceleration instead of a boolean stop")
    p.add_argument("--d-stop", type=float, default=0.01)
    p.add_argument("--d-slow", type=float, default=0.06)
    p.add_argument("--max-step", type=float, default=0.02)
    args = p.parse_args()

    import mujoco
    from skrobot.models import Jedy
    from skrobot.urdf import urdf_to_mjcf

    robot = Jedy()
    robot.reset_pose()
    reset_av = np.array(robot.angle_vector(), dtype=float)
    names = [j.name for j in robot.joint_list]

    # MuJoCo actuator type must match the operating mode.
    atype = {"position": "position", "velocity": "velocity",
             "effort": "motor"}[args.mode]
    mjcf = args.mjcf or "/tmp/jedy_mjcf/jedy_%s.xml" % args.mode
    if not os.path.exists(mjcf):
        # armature/damping stabilise velocity/effort servos (bare URDF inertias
        # are too small for a stiff servo at these timesteps).
        urdf_to_mjcf(robot.urdf_path, mjcf, actuator_type=atype,
                     actuator_kv=2.0, joint_armature=0.05, joint_damping=0.2)
    model = mujoco.MjModel.from_xml_path(mjcf)
    data = mujoco.MjData(model)

    def jid(n):
        return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)

    def aid(n):
        return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n + "_act")

    qadr = {n: model.jnt_qposadr[jid(n)] for n in names if jid(n) >= 0}
    dadr = {n: model.jnt_dofadr[jid(n)] for n in names if jid(n) >= 0}
    act = {n: aid(n) for n in names if jid(n) >= 0}

    # init plant at reset pose; position actuators hold reset, velocity/effort
    # actuators start at 0 (hold still).
    for i, n in enumerate(names):
        if n in qadr:
            data.qpos[qadr[n]] = reset_av[i]
            if act[n] >= 0:
                data.ctrl[act[n]] = reset_av[i] if args.mode == "position" else 0.0
    mujoco.mj_forward(model, data)

    monitor = SelfCollisionMonitor(robot, reference_angle_vector=reset_av)
    arm_reset = [reset_av[names.index(j)] for j in ARM]
    if args.decelerate:
        # per-arm groups so one arm nearing the body doesn't slow the other.
        rlinks = [ln.name for ln in robot.link_list
                  if ln.name.startswith("rarm_link")]
        llinks = [ln.name for ln in robot.link_list
                  if ln.name.startswith("larm_link")]
        scale_groups = {
            "rarm": {"joints": ARM[:7], "links": rlinks},
            "larm": {"joints": ARM[7:], "links": llinks},
        }
        gate = CollisionGate(monitor, ARM, base_angle_vector=reset_av,
                             initial_safe=arm_reset, mode="scale",
                             scale_groups=scale_groups, d_stop=args.d_stop,
                             d_slow=args.d_slow, max_step=args.max_step)
    else:
        gate = CollisionGate(monitor, ARM, base_angle_vector=reset_av,
                             initial_safe=arm_reset, latched=False)

    # attach shm: command stride 3 = [position, velocity, effort] per joint,
    # state stride 2 = [position, velocity]. Created by the C++ ActuoSystem.
    deadline = time.time() + args.timeout
    shm = None
    while time.time() < deadline:
        try:
            shm = ShmBridge(args.shm_name, n_joints=len(ARM), cmd_stride=3,
                            create=False)
            break
        except (FileNotFoundError, ValueError):
            time.sleep(0.2)
    if shm is None:
        print("[jedy_servicer] shm '%s' not ready; is the launch running?"
              % args.shm_name, file=sys.stderr)
        return 1
    # write the reset state immediately so the controllers start from reset
    state = np.zeros((len(ARM), 2), dtype=np.float64)
    for k, j in enumerate(ARM):
        state[k, 0] = arm_reset[k]
    shm.write_state(state)
    # the collision gate operates on position commands only.
    use_gate = (args.mode == "position") and not args.no_gate
    print("[jedy_servicer] attached shm '%s' (%d joints), mode=%s, gate=%s"
          % (args.shm_name, len(ARM), args.mode, "on" if use_gate else "off"),
          flush=True)

    viewer = None
    if args.view:
        import mujoco.viewer
        viewer = mujoco.viewer.launch_passive(model, data)

    dt = 1.0 / args.rate
    substeps = max(1, int(round(dt / model.opt.timestep)))
    stopped_prev = False
    next_t = time.time()
    try:
        while True:
            cmd = shm.read_command()               # flat [pos,vel,eff] * 14
            positions = cmd[0::3]
            velocities = cmd[1::3]
            efforts = cmd[2::3]
            # select the active mode's command; gate only in position mode.
            if args.mode == "position":
                target = gate.filter(positions).applied if use_gate else positions
            elif args.mode == "velocity":
                target = velocities
            else:  # effort
                target = efforts
            for k, j in enumerate(ARM):
                if act[j] >= 0:
                    data.ctrl[act[j]] = target[k]
            for _ in range(substeps):
                mujoco.mj_step(model, data)
            for k, j in enumerate(ARM):
                state[k, 0] = data.qpos[qadr[j]]
                state[k, 1] = data.qvel[dadr[j]]
            shm.write_state(state)

            if use_gate and gate.stopped != stopped_prev:
                stopped_prev = gate.stopped
                if gate.stopped:
                    print("[jedy_servicer] COLLISION-STOP (holding safe pose)",
                          flush=True)
                else:
                    print("[jedy_servicer] cleared", flush=True)

            if viewer is not None:
                if not viewer.is_running():
                    break
                viewer.sync()
            next_t += dt
            sleep = next_t - time.time()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.time()
    except KeyboardInterrupt:
        pass
    finally:
        if viewer is not None:
            viewer.close()
        shm.close()
        print("[jedy_servicer] stopped", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
