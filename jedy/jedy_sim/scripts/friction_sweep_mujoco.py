#!/usr/bin/env python3
"""Release one joint of Jedy in MuJoCo at a series of poses: held or fell?

The MuJoCo counterpart of jedy_skrobot's ``identify_friction``: the same
URDF (expanded jedy_gz.xacro), converted by scikit-robot's ``urdf_to_mjcf``
with ``urdf_dynamics=True`` so the joint's ``<dynamics friction>`` becomes
its ``frictionloss``; the same test (move to the pose, settle, release the
one joint, wait ``--hold`` s, fell = moved more than ``--fell-deg``); and
the same reference torque (scikit-robot ``torque_vector`` at the measured
pose, checked against MuJoCo's own ``qfrc_bias``).

    xacro jedy_gz.xacro gz_control:=false > /tmp/jedy.urdf
    python friction_sweep_mujoco.py /tmp/jedy.urdf --joint rarm_joint0 \\
        --set rarm_joint1=-90 --angles 15 30 40 50 60 90 110 120 135 150

Released = that joint's position actuator gives no force (gain and bias 0),
which is what servo off is on the real robot and in the other simulators.
Every other joint stays held by its actuator. The base is welded to the
world (Gazebo and Isaac stand it on its wheels; the arm's load on the
released joint does not depend on that while the base stays level).
"""

import argparse
import json
import os
import tempfile

import numpy as np


def parse_set(items):
    out = {}
    for item in items or []:
        name, value = item.split('=')
        out[name] = np.deg2rad(float(value))
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('urdf')
    p.add_argument('--joint', required=True)
    p.add_argument('--angles', type=float, nargs='+', required=True,
                   help='poses of the released joint [deg]')
    p.add_argument('--set', nargs='*', default=[],
                   help='other joints, NAME=DEG (the rest are 0)')
    p.add_argument('--settle', type=float, default=1.0,
                   help='held at the pose before the release [s]')
    p.add_argument('--hold', type=float, default=3.0,
                   help='released for this long [s] (identify_friction: 3)')
    p.add_argument('--fell-deg', type=float, default=1.0,
                   help='moved more than this = fell [deg] '
                        '(identify_friction: 1)')
    p.add_argument('--json', default='')
    args = p.parse_args()

    import mujoco
    from skrobot.models.urdf import RobotModelFromURDF
    from skrobot.utils.mjcf_converter import urdf_to_mjcf

    robot = RobotModelFromURDF(urdf_file=args.urdf)
    names = [j.name for j in robot.joint_list]
    out_dir = tempfile.mkdtemp(prefix='jedy_friction_mjcf_')
    mjcf = os.path.join(out_dir, 'jedy.xml')
    urdf_to_mjcf(args.urdf, mjcf, floating_base=False, add_ground=False,
                 urdf_dynamics=True)
    model = mujoco.MjModel.from_xml_path(mjcf)
    data = mujoco.MjData(model)

    def jid(name):
        return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)

    target = jid(args.joint)
    if target < 0:
        raise SystemExit('no joint {} in the MJCF'.format(args.joint))
    dof = model.jnt_dofadr[target]
    qadr = model.jnt_qposadr[target]
    acts = [a for a in range(model.nu)
            if model.actuator_trntype[a] == mujoco.mjtTrn.mjTRN_JOINT
            and model.actuator_trnid[a, 0] == target]
    if len(acts) != 1:
        raise SystemExit('{} has {} actuators, expected 1'.format(
            args.joint, len(acts)))
    act = acts[0]
    gain = model.actuator_gainprm[act].copy()
    bias = model.actuator_biasprm[act].copy()
    tau_f = float(model.dof_frictionloss[dof])

    fixed = parse_set(args.set)
    rows = []
    for angle in args.angles:
        pose = dict(fixed)
        pose[args.joint] = np.deg2rad(angle)
        model.actuator_gainprm[act] = gain
        model.actuator_biasprm[act] = bias
        mujoco.mj_resetData(model, data)
        for a in range(model.nu):
            j = model.actuator_trnid[a, 0]
            jname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
            value = pose.get(jname, 0.0)
            data.qpos[model.jnt_qposadr[j]] = value
            data.ctrl[a] = value
        for name, value in pose.items():
            data.qpos[model.jnt_qposadr[jid(name)]] = value
        mujoco.mj_forward(model, data)
        for _ in range(int(args.settle / model.opt.timestep)):
            mujoco.mj_step(model, data)
        before = float(data.qpos[qadr])

        av = []
        for n in names:
            k = jid(n)
            av.append(float(data.qpos[model.jnt_qposadr[k]]) if k >= 0
                      else 0.0)
        tau_g = float(robot.torque_vector(
            av=np.array(av))[names.index(args.joint)])
        data.qvel[:] = 0.0
        mujoco.mj_forward(model, data)
        tau_mj = float(data.qfrc_bias[dof])

        model.actuator_gainprm[act] = 0.0
        model.actuator_biasprm[act] = 0.0
        for _ in range(int(args.hold / model.opt.timestep)):
            mujoco.mj_step(model, data)
        after = float(data.qpos[qadr])
        moved = np.rad2deg(after - before)
        row = dict(angle=angle, measured_deg=float(np.rad2deg(before)),
                   tau_g=abs(tau_g), tau_g_mujoco=abs(tau_mj),
                   moved_deg=float(moved),
                   fell=bool(abs(moved) > args.fell_deg))
        rows.append(row)
        print('{:+7.1f} deg  tau_g {:.3f} (mujoco {:.3f}) N m  moved {:+8.3f}'
              ' deg  {}'.format(angle, row['tau_g'], row['tau_g_mujoco'],
                                moved, 'FELL' if row['fell'] else 'held'))

    held = [r['tau_g'] for r in rows if not r['fell']]
    fell = [r['tau_g'] for r in rows if r['fell']]
    lower = max(held) if held else None
    upper = min(fell) if fell else None
    print('tau_f in the URDF/MJCF: {:.3f} N m;  bracket: {} <= tau_f <= {}'
          .format(tau_f, lower, upper))
    if args.json:
        with open(args.json, 'w') as f:
            json.dump(dict(backend='mujoco', joint=args.joint, set=args.set,
                           hold=args.hold, tau_f_model=tau_f, rows=rows,
                           lower=lower, upper=upper), f, indent=1)


if __name__ == '__main__':
    main()
