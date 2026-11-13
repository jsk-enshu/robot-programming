#!/usr/bin/env python3
"""Interactive scikit-robot shell for Jedy's whole body (22 joints).

    ros2 launch jedy_bringup jedy_fullbody_all.launch.py  # terminal 1
    ros2 run jedy_skrobot jedy_interface --limb head                  # terminal 2

``--robot`` picks the model, the namespace and the limb set together, because
those have to agree. ``fullbody`` is the only configuration left; the flag is
kept so existing ``--robot fullbody`` command lines keep working.

Binds ``robot`` (the model) and ``ri`` (the live interface, spinning). The same
interactive shell for a robot that has real geometry to look at.

``--viewer rviz2`` draws the model into an RViz already showing the robot,
translucent and on top of it, instead of opening a window of its own. That is
usually the thing being asked: where the arm ends up if this angle vector is
sent, against where it is now.
"""

from __future__ import annotations

import argparse
import os
import sys
import threading

# Before rclpy, so rcutils emits colour on errors in the shell.
os.environ.setdefault("RCUTILS_COLORIZED_OUTPUT", "1")

import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

def _load_robot_module(which):
    """Pick the model/interface pair for a Jedy configuration.

    Parameters
    ----------
    which : str
        'fullbody', the only configuration.

    Returns
    -------
    tuple
        (model class, interface class, CONTROLLED_JOINTS, LIMBS, banner,
        ghost z offset). The last is what the RViz viewer needs to add so a
        model whose URDF disagrees with the robot's still lands on it; 0.0 when
        the two agree.
    """
    if which != "fullbody":
        raise ValueError("unknown Jedy configuration {!r}".format(which))
    import jedy_fullbody_robot_interface as mod
    return (mod.JedyFullbody, mod.JedyFullbodyROS2RobotInterface,
            mod.CONTROLLED_JOINTS, mod.LIMBS, FULLBODY_BANNER,
            mod.GHOST_OFFSET_Z)


def parse_args(argv):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--robot", default="fullbody", choices=["fullbody"],
                   help="which Jedy configuration: 'fullbody', the 22-joint "
                        "robot in /jedy_fullbody brought up by "
                        "jedy_fullbody_all.launch.py (the only one).")
    p.add_argument("--limb", default="right_arm",
                   help="which limb this shell drives. One trajectory "
                        "controller per limb; skrobot's ros2 base keeps ONE "
                        "action per controller type, so a shell drives one "
                        "limb at a time. Run a second shell for a second limb. "
                        "head, right_arm, left_arm, right_gripper or "
                        "left_gripper. Validated against the robot module's "
                        "LIMBS rather than by argparse.")
    p.add_argument("--controller", default=None,
                   help="override the controller name, INCLUDING its "
                        "namespace. See JedyFullbodyROS2RobotInterface for "
                        "why the "
                        "namespace rides here and not in skrobot's namespace "
                        "argument.")
    p.add_argument("--joint-states", default=None,
                   help="joint_states topic (default "
                        "/jedy_fullbody/joint_states). Change it together with "
                        "--controller or the shell reads one robot and "
                        "commands another.")
    p.add_argument("--node-name", default="jedy_skrobot_interface")
    p.add_argument("--controller-timeout", type=float, default=15.0)
    p.add_argument("--viewer", default="pyrender",
                   choices=["pyrender", "rviz2", "none"],
                   help="pyrender opens its own window; rviz2 draws the model "
                        "into an RViz already showing the robot, as a "
                        "translucent ghost, so the pose you are about to "
                        "command sits on top of the pose the robot is in. "
                        "rviz2 needs a MarkerArray display on the topic it "
                        "prints at startup.")
    p.add_argument("--no-viewer", action="store_true",
                   help="same as --viewer none. Kept because it is in "
                        "fingers and in older notes.")
    p.add_argument("--no-check", action="store_true",
                   help="skip comparing this shell's joint list against the "
                        "running servicer. The check needs the servicer up; "
                        "skipping it means a mismatch surfaces later, as a "
                        "trajectory the controller rejects entirely.")
    return p.parse_args(argv)


FULLBODY_BANNER = """\
=== Jedy whole-body scikit-robot shell ===
  robot : JedyFullbody                    (jedy_description, real geometry)
  ri    : JedyFullbodyROS2RobotInterface  (spinning)

  ri.servo_state()                       # enabled / mismatch / would_jump, per joint
  ri.measured()                          # where the robot IS -- unclamped truth
  ri.outside_model_limits()              # {} unless the model is lying to you
  ri.servo_on()                          # re-syncs the controller first, then holds

  robot.angle_vector()                   # model's pose (edit, then send)
  ri.angle_vector(robot.angle_vector(), 5.0); ri.wait_interpolation()
  ri.servo_off()                         # SUPPORT THE ARMS -- they go limp

  robot.head_joint0.joint_angle(0.3)            # one joint, in the model
  ri.angle_vector(robot.angle_vector(), 3.0)    # ...then on the robot

This shell drives ONE limb: --limb head | right_arm | left_arm |
right_gripper | left_gripper; `ri.limb` says which. The five controllers claim
disjoint interfaces, so they are all active at once and a head goal does not
replace an arm's trajectory -- run a second shell for a second limb.

! ALL 22 SERVOS ARE ON THE BUS (board scan 2026-08-03), so there is no joint
! here that the model can move and the robot cannot.
! The exceptions are structural rather than missing hardware:
!   - the four mecanum wheels are continuous joints in VELOCITY mode, so they
!     are in no trajectory. Drive the base with a twist on /cmd_vel.
!   - rarm_mimic_gripper_joint / larm_mimic_gripper_joint carry a URDF <mimic>
!     and have no servo. They follow their driven side, in the model and in TF.

! potentio_vector() CLAMPS to the URDF limits and does not say so. If a joint
! sits outside its limits the model reports the limit, not the truth -- use
! ri.measured(). ri.outside_model_limits() tells you when this is live.
"""


def main(argv=None):
    if argv is None:
        argv = sys.argv[1:]
    if "--ros-args" in argv:
        cut = argv.index("--ros-args")
        cli_argv, ros_argv = argv[:cut], argv[cut:]
    else:
        cli_argv, ros_argv = argv, []
    args = parse_args(cli_argv)

    rclpy.init(args=ros_argv or None)

    model_cls, interface_cls, CONTROLLED_JOINTS, limbs, banner, ghost_offset = \
        _load_robot_module(args.robot)
    if args.limb not in limbs:
        raise SystemExit(
            "--limb {!r} is not a limb of the {} robot. It has: {}".format(
                args.limb, args.robot, ", ".join(sorted(limbs))))

    robot = model_cls()
    kwargs = dict(
        node_name=args.node_name,
        controller_timeout=args.controller_timeout,
    )
    if args.controller is not None:
        kwargs["controller_name"] = args.controller
    if args.joint_states is not None:
        kwargs["joint_states_topic"] = args.joint_states
    ri = interface_cls(robot=robot, limb=args.limb, **kwargs)

    if not args.no_check:
        # Before anything is commanded, not after. See check_against_servicer.
        #
        # And BEFORE the executor below starts, which is not a style choice.
        # This goes through wait_for_message(), which spins the node itself to
        # collect one latched message, and that function's docstring is explicit
        # that it is "only safe on a node NO executor owns": spinning a node a
        # background executor is already spinning corrupts the shared wait set.
        # It surfaces as
        #
        #   RCLError: Failed to get number of ready entities for action client:
        #   wait set index for status subscription is out of bounds
        #
        # raised on the EXECUTOR thread, which then dies. Nothing else does --
        # the shell comes up, the banner prints, `ri` looks fine -- and then the
        # first ri.angle_vector() blocks forever in wait_interpolation(), because
        # nothing is spinning any more. Observed on 2026-08-03 with the
        # full-body robot when the check ran after the executor had started.
        driven = ri.check_against_servicer()
        ri.get_logger().info(
            "servicer agrees: {} joints, in this order".format(len(driven)))

    executor = SingleThreadedExecutor()
    executor.add_node(ri)
    threading.Thread(target=executor.spin, daemon=True).start()

    which_viewer = "none" if args.no_viewer else args.viewer
    viewer = None
    if which_viewer != "none":
        try:
            if which_viewer == "rviz2":
                from rviz2_viewer import RViz2Viewer
                # Shares the interface's node, so the markers ride the executor
                # that is already spinning above rather than needing a second.
                #
                # Stood beside the robot rather than on top of it. Overlaid,
                # the two are legible only while the poses are close, and the
                # moment they are not -- which is the moment worth looking at --
                # the ghost and the robot interpenetrate into something neither
                # of them is. Offset in -y, so it is on the robot's right in the
                # default camera.
                #
                # The z term is NOT taste: it cancels the base_link the model's
                # own URDF uses against the one the robot publishes. See
                # GHOST_OFFSET_Z, which records the measurement and the fact
                # that it patches a real disagreement rather than fixing one.
                # Move the lot with viewer.offset = (0, 0, 0).
                viewer = RViz2Viewer(
                    node=ri, offset=(0.0, -0.5, ghost_offset))
            else:
                from skrobot.viewers import PyrenderViewer
                viewer = PyrenderViewer()
            viewer.add(robot)
            viewer.show()
        except Exception as err:                       # noqa: BLE001
            # A headless box must still get the shell; the viewer is a
            # convenience, and losing it silently would be worse than saying so.
            viewer = None
            ri.get_logger().warn(
                "no viewer ({}); continuing without it".format(err))

    # Open the shell with the model already on the real pose, so a first
    # `ri.angle_vector(robot.angle_vector(), t)` holds rather than moves.
    #
    # Synced from ri.measured() -- the servicer's own per-joint numbers, by
    # name -- and NOT from ri.angle_vector(). The obvious version,
    #
    #     actual = ri.angle_vector()
    #     if len(actual) == len(robot.joint_list): robot.angle_vector(actual)
    #
    # is the obvious version and it silently did nothing here: it
    # depends on a joint_states message having arrived in the moment between
    # starting the executor and asking, and when it has not, the guard fails
    # quietly and the model stays at zero. Measured on this robot: the shell
    # opened with the model at 0 while the arms were at -5, +4, -18, +82 deg,
    # so the very first `ri.angle_vector(robot.angle_vector(), 5.0)` -- the
    # line the banner offers -- would have driven every joint to zero. A sync
    # that fails open is worse than no sync, because the banner promises one.
    #
    # ri.robot is a deepcopy, so nothing here reaches the interface's own model.
    try:
        measured = ri.measured()
        for name, value in measured.items():
            joint = getattr(robot, name, None)
            if joint is not None:
                joint.joint_angle(value)
        stale = [n for n in CONTROLLED_JOINTS if n not in measured]
        if stale:
            ri.get_logger().warn(
                "no measurement for {}; the model shows 0 for "
                "them".format(", ".join(stale)))
        # joint_angle() CLAMPS into the URDF limits and does not say so, so a
        # joint parked outside them leaves the model showing the limit. Say it.
        outside = ri.outside_model_limits(measured)
        if outside:
            ri.get_logger().warn(
                "the model CANNOT represent {}: the URDF limits clamp them, so "
                "robot.angle_vector() is not what the robot is doing. Use "
                "ri.measured().".format(
                    ", ".join("{} at {:.3f} rad".format(n, v[0])
                              for n, v in outside.items())))
    except Exception as err:                           # noqa: BLE001
        ri.get_logger().warn(
            "could not sync the model to the robot at startup: {} -- "
            "robot.angle_vector() is NOT where the robot is; do not send it "
            "without checking ri.measured() first.".format(err))

    local = dict(robot=robot, ri=ri, viewer=viewer,
                 CONTROLLED_JOINTS=CONTROLLED_JOINTS)
    try:
        from IPython import embed
        embed(header=banner, colors="neutral", user_ns=local)
    except ImportError:
        import code
        code.interact(banner=banner, local=local)
    finally:
        executor.shutdown()
        ri.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
