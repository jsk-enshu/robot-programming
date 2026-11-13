#!/usr/bin/env python3
"""Find a joint's static friction from where it holds and where it falls.

A servo with its torque off is held by the friction of its gear train,
tau_f. Put the joint in a pose, release it alone, and watch it: if the
gravity torque on it there, tau_g, is below tau_f it stays; above, it falls.
Doing that over poses whose tau_g spans tau_f brackets it:

    max tau_g that held  <=  tau_f  <=  min tau_g that fell

    ros2 run jedy_skrobot identify_friction --joint rarm_joint0 \\
        --angles 10 20 30 40 60 90
    ros2 run jedy_skrobot identify_friction --joint rarm_joint1 \\
        --set rarm_joint0=90 --angles -15 -30 -45 -60 -90

tau_g is computed by scikit-robot (``RobotModel.torque_vector``) from the
URDF the robot is running (``robot_description``, so the same masses the
simulators use) at the MEASURED pose just before the release. The base is
assumed level.

``--write XACRO`` puts the midpoint of the bracket into that joint's
``<dynamics friction=...>`` in XACRO (jedy_description/urdf/jedy_gz.xacro)
and rewrites the joint's ``source:`` comment line with the measurement. It
refuses when the bracket is open (nothing held, or nothing fell) or
inconsistent (something held at a larger tau_g than something that fell).

The same script runs on Gazebo, Isaac Sim and the real robot; on the real
robot keep a hand under the arm (HUMAN_CHECKLIST.md).
"""

import argparse
import datetime
import json
import re
import threading

import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from std_msgs.msg import String

from jedy_skrobot import connect
from jedy_skrobot.interface import NAMESPACE

#: A joint that moved less than this while released "held" [deg].
FELL_DEG = 1.0


def robot_description(timeout=10.0):
    """The URDF the running bring-up published (transient local)."""
    node = rclpy.create_node('identify_friction_description')
    got = {}
    done = threading.Event()

    def on_msg(msg):
        got['urdf'] = msg.data
        done.set()

    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, '/{}/robot_description'.format(
        NAMESPACE.strip('/')), on_msg, qos)
    deadline = node.get_clock().now().nanoseconds + timeout * 1e9
    while not done.is_set() and \
            node.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    if 'urdf' not in got:
        raise SystemExit('no robot_description within {} s'.format(timeout))
    return got['urdf']


def gravity_model(urdf_text):
    from skrobot.models.urdf import RobotModelFromURDF
    return RobotModelFromURDF(urdf=urdf_text)


#: torque_vector and the derivative of the potential energy must agree to
#: this [N m]; they differ by far more when torque_vector is wrong.
TAU_CHECK_TOL = 1e-3


def potential_energy(model, q):
    """Gravitational potential energy of ``model`` at ``q`` [J]."""
    model.angle_vector(q)
    energy = 0.0
    for link in model.link_list:
        mass = getattr(link, 'mass', None)
        if not mass or link.centroid is None:
            continue
        com = link.worldpos() + link.worldrot().dot(np.asarray(link.centroid))
        energy += mass * 9.80665 * com[2]
    return energy


def gravity_torque(model, names, av, joint):
    """|tau_g| on ``joint`` [N m] at angle vector ``av`` (``names`` order).

    Checked against the numerical derivative of the potential energy on
    that joint: scikit-robot 0.4.6 (477a533) permuted torque_vector on
    branched trees and gave Jedy's joints each other's torques (fixed in
    later scikit-robot). A wrong reference
    torque silently shifts every bracket, so refuse instead.
    """
    mnames = [j.name for j in model.joint_list]
    q = np.array([av[names.index(n)] if n in names else 0.0
                  for n in mnames])
    i = mnames.index(joint)
    # skrobot clips angle_vector to the limits; measure at the clipped pose
    # and difference inwards when it sits on a limit.
    j = model.joint_list[i]
    q[i] = np.clip(q[i], j.min_angle, j.max_angle)
    tau = float(model.torque_vector(av=q.copy())[i])
    step = 1e-6
    up = min(step, j.max_angle - q[i])
    down = min(step, q[i] - j.min_angle)
    dq_up = np.zeros(len(q))
    dq_up[i] = up
    dq_down = np.zeros(len(q))
    dq_down[i] = down
    reference = (potential_energy(model, q + dq_up)
                 - potential_energy(model, q - dq_down)) / (up + down)
    model.angle_vector(q)
    if abs(tau - reference) > TAU_CHECK_TOL:
        raise SystemExit(
            'torque_vector gives {:+.4f} N m on {} but dU/dq is {:+.4f}: '
            'this scikit-robot computes gravity torques wrongly (needs a '
            'scikit-robot whose torque_vector handles branched trees)'
            .format(tau, joint, reference))
    return abs(tau)


def bracket(rows):
    """(lower, upper) bound of tau_f from rows of dict(tau_g, fell)."""
    held = [r['tau_g'] for r in rows if not r['fell']]
    fell = [r['tau_g'] for r in rows if r['fell']]
    lower = max(held) if held else None
    upper = min(fell) if fell else None
    return lower, upper


def write_xacro(path, joint, tau_f, rows, lower, upper, backend):
    """Put ``tau_f`` into ``joint``'s <dynamics friction> in ``path``."""
    text = open(path).read()
    block = re.search(r'  <joint name="{}" type="\w+">\n.*?  </joint>\n'
                      .format(re.escape(joint)), text, re.S)
    if block is None:
        raise SystemExit('{}: no joint {}'.format(path, joint))
    body = block.group(0)
    new, n = re.subn(r'(<dynamics damping="[^"]*" friction=")[^"]*(")',
                     r'\g<1>{:.3g}\g<2>'.format(tau_f), body)
    if n != 1:
        raise SystemExit('{}: joint {} has no <dynamics damping=... '
                         'friction=...> to rewrite'.format(path, joint))
    note = ('source: measured {} on {} with identify_friction: held at '
            'tau_g <= {:.3f}, fell at tau_g >= {:.3f} N m ({} poses); '
            'tau_f = midpoint.'.format(
                datetime.date.today().isoformat(), backend, lower, upper,
                len(rows)))
    new, n = re.subn(r'source: [^\n]*?(-->)?\n',
                     lambda m: note + (' -->' if m.group(1) else '') + '\n',
                     new, count=1)
    if n != 1:
        raise SystemExit('{}: joint {} has no "source:" comment line'
                         .format(path, joint))
    text = text[:block.start()] + new + text[block.end():]
    open(path, 'w').write(text)


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument('--joint', required=True)
    p.add_argument('--angles', type=float, nargs='+', required=True,
                   help='angles of --joint to release it at [deg]')
    p.add_argument('--set', nargs='*', default=[],
                   help='other joints for every pose, NAME=DEG')
    p.add_argument('--hold', type=float, default=3.0,
                   help='seconds (robot time) to leave it released')
    p.add_argument('--move-time', type=float, default=2.0)
    p.add_argument('--servo-timeout', type=float, default=10.0,
                   help='wall-clock seconds to wait for servo on/off; a '
                        'simulator running slower than real time needs more')
    p.add_argument('--backend', default='unknown',
                   help='a label for the log and the xacro comment')
    p.add_argument('--json', default=None, help='write the rows here')
    p.add_argument('--write', default=None, metavar='XACRO',
                   help='put the bracket midpoint into this xacro')
    args = p.parse_args()

    robot, ri = connect()
    names = [j.name for j in robot.joint_list]
    if args.joint not in names:
        raise SystemExit('no joint {}'.format(args.joint))
    model = gravity_model(robot_description())
    fixed = {}
    for item in args.set:
        name, deg = item.split('=')
        fixed[name] = np.deg2rad(float(deg))

    start = np.array(ri.angle_vector())
    rows = []
    print('{:>9s} {:>9s} {:>9s} {:>9s}  {}'.format(
        'angle', 'tau_g', 'before', 'after', ''))
    for deg in args.angles:
        av = start.copy()
        for name, rad in fixed.items():
            av[names.index(name)] = rad
        av[names.index(args.joint)] = np.deg2rad(deg)
        ri.angle_vector(av, args.move_time)
        ri.wait_interpolation()
        ri.sleep(0.5)
        measured = np.array(ri.angle_vector())
        tau_g = gravity_torque(model, names, measured, args.joint)
        before = np.rad2deg(measured[names.index(args.joint)])
        ri.servo_off(joints=[args.joint], timeout=args.servo_timeout)
        ri.sleep(args.hold)
        after = np.rad2deg(ri.angle_vector()[names.index(args.joint)])
        ri.servo_on(joints=[args.joint], timeout=args.servo_timeout)
        fell = abs(after - before) >= FELL_DEG
        rows.append(dict(angle=float(deg), tau_g=float(tau_g),
                         before=float(before), after=float(after),
                         sag=float(after - before), fell=bool(fell)))
        print('{:9.1f} {:9.3f} {:9.2f} {:9.2f}  {}'.format(
            deg, tau_g, before, after,
            'FELL {:+.1f} deg'.format(after - before) if fell else 'held'),
            flush=True)
    ri.angle_vector(start, args.move_time)
    ri.wait_interpolation()

    lower, upper = bracket(rows)
    consistent = lower is None or upper is None or lower <= upper
    print('{}: held up to tau_g = {}, fell from tau_g = {} N m{}'.format(
        args.joint,
        'n/a' if lower is None else '{:.3f}'.format(lower),
        'n/a' if upper is None else '{:.3f}'.format(upper),
        '' if consistent else '  INCONSISTENT (held above where it fell)'))
    if args.json:
        with open(args.json, 'w') as f:
            json.dump(dict(joint=args.joint, backend=args.backend,
                           set=args.set, hold=args.hold, rows=rows,
                           lower=lower, upper=upper,
                           consistent=consistent), f, indent=1)
    if args.write:
        if lower is None or upper is None:
            raise SystemExit('not written: the bracket is open (add poses '
                             'that {})'.format(
                                 'hold' if lower is None else 'fall'))
        if not consistent:
            raise SystemExit('not written: inconsistent bracket')
        tau_f = 0.5 * (lower + upper)
        write_xacro(args.write, args.joint, tau_f, rows, lower, upper,
                    args.backend)
        print('wrote friction="{:.3g}" for {} into {}'.format(
            tau_f, args.joint, args.write))
    if not consistent:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
