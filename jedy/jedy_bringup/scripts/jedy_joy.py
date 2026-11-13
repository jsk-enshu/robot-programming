#!/usr/bin/env python3
"""Drive Jedy from a PS4 pad: the base, the head, and torque on and off.

    ros2 launch jedy_bringup jedy_joy.launch.py

    L1 (hold)          deadman. Nothing moves without it, and letting go stops
                       the base within one command period.
    left stick         drive: up/down = forward/back, left/right = strafe
    right stick left/right   turn
    D-pad              head: left/right = pan, up/down = tilt
    L2 (hold)          head at twice its rate. It does nothing for the base,
                       which already drives at its top speed unmodified
    R1 (hold) + stick  slow mode, as slow as this base can go
    L1+R1+cross, held  go to the HOME pose over --home-time seconds

    circle             RELEASE all torque. Works WITHOUT the deadman, because
                       the one button you reach for in a hurry must not need a
                       second one held down.
    triangle           torque ON. Re-syncs the arm controllers first (below).

Three things here are about feel rather than function, and each is a
consequence of something measured on this robot.

**The stick is mapped onto what the base can do, not onto 0..max.** The wheels
do not turn below about 0.071 m/s and saturate near 0.173, so a plain
proportional stick leaves the bottom 42 % of its travel dead and then lurches.
:mod:`jedy_teleop.deadband` scales the whole twist -- preserving every ratio,
because on a mecanum base the ratio between wheels IS the heading -- so any
deliberate deflection produces motion and none of it produces the wrong
direction.

**L2 speeds up the head and NOT the base, because the base has no faster to
go.** A full stick already asks for `max_wheel_rad_s` and `scale_to_usable`
caps there -- past that limit one wheel saturates while the others keep their
speed, which does not make the base slower, it aims it somewhere else. A fast
button for the base could therefore only work by making the plain stick
slower first, and an unmodified stick that is not the top speed is a worse
default than no fast button at all. The head has the opposite shape:
`--head-rate` is a number someone chose, so there L2 is a real increase, and
`--head-lead` bounds what happens when it asks for more than the servo gives.

**Torque ON re-syncs the controllers first, or the button would just fail.**
Enabling is refused unless the controller already commands where the robot is,
and after the robot has been hand-moved it does not. The RViz panel solves this
by sending each trajectory controller a goal to the pose the robot is ALREADY
in -- a move that moves nothing whose only effect is to put the setpoint on the
measured pose -- and this does the same. Without it the button reads as broken
exactly when an operator most wants it.

**Going home takes a held combination, not a button.** A whole-body move over
several seconds is not something a pad brushed against a table should start, so
it needs both shoulders and cross, held for a second, and it says so while the
second is counting. It also refuses if any of the joints involved is still
limp: a trajectory to a joint with no torque does nothing, and the rest of the
robot arriving without it is not the pose anyone asked for. The pose itself is
in jedy_bringup's config, per robot, with its grouping by controller written
out rather than inferred from joint-name prefixes.

**The base and the head share one deadman but not one stick.** Driving while
looking somewhere else is the normal case for a mobile robot, so the head is on
the D-pad where it can be nudged without letting go of anything.
"""

from __future__ import annotations

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import rclpy  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (  # noqa: E402
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy)

from builtin_interfaces.msg import Duration  # noqa: E402
from control_msgs.action import FollowJointTrajectory  # noqa: E402
from geometry_msgs.msg import TwistStamped  # noqa: E402
from sensor_msgs.msg import Joy, JointState  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from trajectory_msgs.msg import (  # noqa: E402
    JointTrajectory, JointTrajectoryPoint)
from actuo_msgs.action import SetServo  # noqa: E402
from actuo_msgs.msg import ServoState  # noqa: E402

from jedy_teleop.deadband import scale_to_usable  # noqa: E402
from jedy_teleop.mapping import BUTTON_CROSS  # noqa: E402

from jedy_teleop.mapping import (  # noqa: E402
    AXIS_DPAD_X, AXIS_DPAD_Y, AXIS_LEFT_X, AXIS_LEFT_Y, AXIS_RIGHT_X,
    BUTTON_CIRCLE, BUTTON_L1, BUTTON_L2, BUTTON_R1, BUTTON_TRIANGLE)

#: Fallback head joints, used only when --head-joints is not given: Jedy's
#: pan and tilt joints, as jedy_fullbody_all.launch.py passes them.
DEFAULT_HEAD_JOINTS = "head_joint0,head_joint1"


def parse_args(argv):
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--namespace", default="/jedy_fullbody")
    p.add_argument("--servicer", default="/actuo_bus_servicer")
    p.add_argument("--head-joints", default=DEFAULT_HEAD_JOINTS,
                   help="comma-separated, pan first then tilt. The D-pad "
                        "drives exactly these two, so the order is the "
                        "mapping and not a formality.")
    # jedy_fullbody's base: the wheels' collision cylinders in the URDF, and
    # |x| + |y| = 0.0435 + 0.0505 for a wheel from the centre.
    p.add_argument("--wheel-radius", type=float, default=0.02525)
    p.add_argument("--centre-projection", type=float, default=0.094)
    # Measured, and the reason the stick is remapped at all. The defaults are
    # the RCB-4mini config's (jedy_fullbody_rcb4.yaml).
    p.add_argument("--min-wheel-rad-s", type=float, default=3.09,
                   help="the deadband, as a wheel speed: 5 command deg at "
                        "~0.0985 rev/s per deg. Below this a wheel does not "
                        "turn at a predictable speed.")
    p.add_argument("--max-wheel-rad-s", type=float, default=6.19,
                   help="top speed, ~0.98 rev/s (10 command deg)")
    p.add_argument("--slow-scale", type=float, default=0.55,
                   help="R1 factor. Not smaller: the deadband puts a floor "
                        "under how slowly this base can go at all, and a slow "
                        "mode that asks for less than that just stops.")
    p.add_argument("--head-rate", type=float, default=1.0,
                   help="rad/s the D-pad moves a head joint")
    p.add_argument("--head-boost", type=float, default=2.0,
                   help="L2 factor for the head. head-rate is a number "
                        "someone chose and not a measured ceiling, so unlike "
                        "the base this is a real increase. The ceiling that "
                        "does exist is --head-lead: the target may never run "
                        "further than that ahead of the measured angle, so "
                        "rate * boost has to stay under head-lead * --rate "
                        "(4.0 rad/s as these default) or a held D-pad spends "
                        "every cycle against the anti-windup instead of "
                        "moving. Checked at startup.")
    p.add_argument("--head-lead", type=float, default=0.20,
                   help="rad the head target may run ahead of the measured "
                        "angle before the D-pad stops adding to it. Without "
                        "this, holding the pad while the head is blocked or "
                        "its torque is off winds the target up somewhere the "
                        "head is not -- and it snaps there the moment it can "
                        "move. Anti-windup, and it is the same reason a "
                        "controller has one.")
    p.add_argument("--rate", type=float, default=20.0)
    p.add_argument("--poses", default="",
                   help="named poses YAML (default: jedy_bringup's). Holds the "
                        "angles AND which controller owns which joint, because "
                        "a trajectory naming joints a controller does not own "
                        "is rejected outright rather than partially applied.")
    p.add_argument("--home-time", type=float, default=5.0,
                   help="seconds the move to the home pose takes")
    p.add_argument("--home-hold", type=float, default=1.0,
                   help="seconds the button combination must be HELD. A "
                        "whole-body move is not something a brushed button "
                        "should start, so it takes a deliberate input and a "
                        "second of it.")
    p.add_argument("--joy-timeout", type=float, default=0.3,
                   help="seconds without a /joy message before the pad counts "
                        "as gone and everything is treated as released. "
                        "joy_node autorepeats at 20 Hz while a pad is "
                        "connected, so this is about six missed messages.")
    p.add_argument("--ros-args", dest="ros_args", nargs=argparse.REMAINDER,
                   default=[])
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    rclpy.init(args=args.ros_args or None)
    node = Node("jedy_joy")

    head_joints = [j.strip() for j in args.head_joints.split(",") if j.strip()]
    if len(head_joints) != 2:
        raise SystemExit(
            "--head-joints needs exactly two joints, pan then tilt; got {!r}"
            .format(args.head_joints))

    twist_pub = node.create_publisher(TwistStamped, "/cmd_vel", 10)
    servo = ActionClient(node, SetServo, "{}/servo/set".format(args.servicer))
    head_pub = node.create_publisher(
        JointTrajectory, "{}/head_controller/joint_trajectory".format(
            args.namespace), 10)

    state = {"joy": None, "joy_stamp": None, "lost": False, "measured": {},
             "last_note": None, "head_goal": None, "enable_in": 0,
             "limits": {}, "warned_limit": set(), "home_held": 0.0,
             "home_sent": False, "torque_off": set(), "driving": False}

    poses_path = args.poses
    if not poses_path:
        from ament_index_python.packages import get_package_share_directory
        poses_path = os.path.join(
            get_package_share_directory("jedy_bringup"), "config",
            "jedy_fullbody_poses.yaml")
    import yaml
    with open(poses_path) as handle:
        poses = yaml.safe_load(handle)["poses"]
    node.get_logger().info("poses from {}: {}".format(
        poses_path, ", ".join(sorted(poses))))

    # Which controller owns which joint, for the re-sync that TRIANGLE does
    # before enabling torque. Taken from the `home` pose's own grouping rather
    # than guessed from joint-name prefixes.
    #
    # A prefix rule is not merely inelegant here, it is wrong on any robot whose
    # limb has more than one controller: jedy_fullbody splits the grippers out,
    # so rarm_gripper_joint starts with "rarm_" but belongs to
    # right_gripper_controller, and naming it in a right_arm_controller goal
    # gets the WHOLE goal rejected -- taking the seven real arm joints down with
    # it, silently, right before a torque enable that then fails its guard.
    #
    # The poses file already states this grouping, go_home already trusts it,
    # and its own header says why it is written out instead of inferred.
    resync_groups = {controller: list(joints)
                     for controller, joints in poses.get("home", {}).items()}
    if not resync_groups:
        node.get_logger().warn(
            "no 'home' pose in {}: TRIANGLE cannot re-sync the controllers "
            "before enabling torque, so the enable will be refused whenever "
            "the robot has drifted from what they command".format(poses_path))
    resync = {name: ActionClient(
        node, FollowJointTrajectory,
        "{}/{}/follow_joint_trajectory".format(args.namespace, name))
        for name in resync_groups}

    pose_clients = {}

    def go_home():
        """Drive every joint of the `home` pose there, one goal per controller.

        Refuses rather than half-moving if a joint is limp: a trajectory to a
        joint with no torque does nothing, and the rest of the robot moving
        while one limb hangs is not the pose anyone asked for.
        """
        pose = poses.get("home")
        if not pose:
            node.get_logger().error("no 'home' pose in the poses file")
            return
        wanted = [j for group in pose.values() for j in group]
        limp = [j for j in wanted if j in state["torque_off"]]
        if limp:
            node.get_logger().error(
                "not going home: {} still limp. Press TRIANGLE first -- a "
                "trajectory to a joint with no torque does nothing, and the "
                "rest of the robot arriving without it is not the pose you "
                "asked for.".format(", ".join(limp)))
            return
        node.get_logger().warn(
            "GOING HOME over {:.0f}s -- {} joints are about to move".format(
                args.home_time, len(wanted)))
        for controller, joints in pose.items():
            action = "{}/{}/follow_joint_trajectory".format(
                args.namespace, controller)
            client = pose_clients.get(action)
            if client is None:
                client = ActionClient(node, FollowJointTrajectory, action)
                pose_clients[action] = client
            if not client.server_is_ready():
                node.get_logger().error(
                    "no controller at {} -- {} did NOT move".format(
                        action, ", ".join(joints)))
                continue
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(joints)
            point = JointTrajectoryPoint()
            point.positions = [float(v) for v in joints.values()]
            point.time_from_start = Duration(
                sec=int(args.home_time),
                nanosec=int((args.home_time % 1.0) * 1e9))
            goal.trajectory.points = [point]
            client.send_goal_async(goal)

    def on_description(msg):
        """Joint limits, read from the robot's own URDF.

        Not a constant in this file, and not optional. The head target is an
        integrator: hold the D-pad long enough and it walks past the joint's
        limit, at which point the actuator refuses the angle by RAISING inside
        the servicer's control cycle -- which takes the servicer down and every
        other joint's telemetry with it. Clamping here keeps a held button from
        being able to do that.
        """
        import xml.etree.ElementTree as ET
        try:
            root = ET.fromstring(msg.data)
        except ET.ParseError as error:                     # noqa: BLE001
            node.get_logger().error(
                "could not parse robot_description ({}); the head will not be "
                "clamped to its joint limits and a held D-pad could walk it "
                "past them".format(error))
            return
        for joint in root.findall("joint"):
            limit = joint.find("limit")
            if limit is None or limit.get("lower") is None:
                continue
            state["limits"][joint.get("name")] = (
                float(limit.get("lower")), float(limit.get("upper")))
        node.get_logger().info(
            "read joint limits for {} joints".format(len(state["limits"])))

    node.create_subscription(
        String, "{}/robot_description".format(args.namespace), on_description,
        QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                   reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def on_joint_states(msg):
        state["measured"].update(zip(msg.name, msg.position))

    def on_servo_state(msg):
        state["torque_off"] = {j.name for j in msg.joints if not j.enabled}

    node.create_subscription(
        ServoState, "{}/servo/state".format(args.servicer), on_servo_state,
        QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                   reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL))

    node.create_subscription(JointState,
                             "{}/joint_states".format(args.namespace),
                             on_joint_states, 10)

    def _send_servo(enable):
        """Ask, and SAY WHAT CAME BACK.

        The first version fired the goal and never looked at the result, so a
        refused enable was indistinguishable from a button that did nothing --
        and the server refuses for good reasons an operator can act on ("this
        joint is outside its taught limit", "the controller commands somewhere
        else"). A button that swallows the answer is worse than one that does
        not exist: it teaches you the feature is broken when the robot is
        telling you exactly what is wrong.
        """
        future = servo.send_goal_async(SetServo.Goal(enable=enable))

        def on_accepted(goal_future):
            handle = goal_future.result()
            if handle is None or not handle.accepted:
                node.get_logger().error(
                    "servo {} was not even accepted -- is the servicer "
                    "running?".format("ON" if enable else "OFF"))
                return

            def on_result(result_future):
                outcome = result_future.result().result
                if outcome.ok:
                    node.get_logger().info("servo: " + outcome.reason)
                else:
                    node.get_logger().error("servo REFUSED: " + outcome.reason)

            handle.get_result_async().add_done_callback(on_result)

        future.add_done_callback(on_accepted)

    def pressed(msg, previous, index):
        """A rising edge. Buttons are polled, so without this a single press
        fires for every cycle it is held -- which for `release torque` is
        harmless and for `torque on` is a stream of goals."""
        return (index < len(msg.buttons) and msg.buttons[index]
                and not (previous and index < len(previous.buttons)
                         and previous.buttons[index]))

    def send_resync():
        """Ask each arm controller for a trajectory to where its joints already
        are. A move that moves nothing, whose only effect is to leave the
        setpoint on the measured pose -- which is what the enable guard needs.
        """
        for name, client in resync.items():
            if not client.server_is_ready():
                continue
            # Intersected with what is actually being reported: a joint the
            # pose names but the broadcaster does not publish has no measured
            # value to command, and naming it would get the goal rejected.
            joints = [j for j in resync_groups[name] if j in state["measured"]]
            if not joints:
                continue
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = joints
            point = JointTrajectoryPoint()
            point.positions = [state["measured"][j] for j in joints]
            point.time_from_start = Duration(sec=0, nanosec=200_000_000)
            goal.trajectory.points = [point]
            client.send_goal_async(goal)

    def on_joy(msg):
        previous = state["joy"]
        state["joy"] = msg
        state["joy_stamp"] = node.get_clock().now().nanoseconds * 1e-9
        if state["lost"]:
            state["lost"] = False
            node.get_logger().info("pad is back")

        if pressed(msg, previous, BUTTON_CIRCLE):
            # No deadman on this one. The button you reach for in a hurry must
            # not need a second one held down.
            _send_servo(False)
            node.get_logger().warn("RELEASE: torque off, everything goes limp")

        if pressed(msg, previous, BUTTON_TRIANGLE):
            send_resync()
            # Enable on a LATER tick, not now: it is judged against what the
            # controllers are commanding, and they have just been asked to
            # adopt the measured pose. Counted down in tick() rather than done
            # with a one-shot timer, because a timer that has to destroy itself
            # from inside its own callback is a lifecycle bug waiting to
            # happen and this needs none of it.
            state["enable_in"] = int(0.4 * args.rate)
            node.get_logger().info(
                "re-syncing the controllers, then torque ON")

    node.create_subscription(Joy, "/joy", on_joy, 10)

    def tick():
        if state["enable_in"] > 0:
            state["enable_in"] -= 1
            if state["enable_in"] == 0:
                _send_servo(True)

        # The pad going away must look like every button released, and it does
        # NOT do so on its own. joy_node stops publishing when the device is
        # removed -- verified in its source: handleJoyDeviceRemoved empties the
        # message and clears the handle, and the publish is guarded on that
        # handle -- so /joy simply goes quiet. Without this check the last
        # message would keep being read, and a pad unplugged mid-drive with the
        # deadman and the stick held would leave the base running on it
        # forever. mecanum_drive_controller's own reference_timeout does not
        # save it either: this node would still be publishing.
        msg = state["joy"]
        if msg is not None and state["joy_stamp"] is not None:
            age = node.get_clock().now().nanoseconds * 1e-9 - state["joy_stamp"]
            if age > args.joy_timeout:
                if not state["lost"]:
                    state["lost"] = True
                    node.get_logger().warn(
                        "no /joy for {:.1f}s -- treating the pad as gone: the "
                        "base stops and the head holds. It will pick up by "
                        "itself when the pad is back; joy_node reopens the "
                        "device on its own.".format(age))
                msg = None
                state["head_goal"] = None
        twist = TwistStamped()
        twist.header.stamp = node.get_clock().now().to_msg()
        # The home combination: both shoulders AND cross, held. Deliberately
        # awkward -- a whole-body move over several seconds is not something a
        # button brushed against a table should start. Checked here rather than
        # in the joy callback because a HOLD is a property of time, not of an
        # edge, and checked AFTER the watchdog above rather than against the
        # last message received: a pad that vanishes mid-combination must not
        # leave a held state behind that keeps counting up to a whole-body
        # move nobody is holding a button for any more.
        held = msg
        if (held is not None and len(held.buttons) > max(BUTTON_R1, BUTTON_CROSS)
                and held.buttons[BUTTON_L1] and held.buttons[BUTTON_R1]
                and held.buttons[BUTTON_CROSS]):
            state["home_held"] += 1.0 / args.rate
            if state["home_held"] >= args.home_hold and not state["home_sent"]:
                state["home_sent"] = True
                go_home()
        else:
            if state["home_held"] > 0.0 and not state["home_sent"]:
                node.get_logger().info(
                    "home combination released after {:.1f}s of the {:.1f}s "
                    "needed -- nothing was sent".format(
                        state["home_held"], args.home_hold))
            state["home_held"] = 0.0
            state["home_sent"] = False

        driving = (msg is not None and len(msg.buttons) > BUTTON_L1
                   and msg.buttons[BUTTON_L1]
                   and not state["home_held"])
        if driving:
            # Two speeds, and L2 is not one of them: the unmodified stick is
            # already the top speed. See the module docstring.
            scale = (args.slow_scale
                     if len(msg.buttons) > BUTTON_R1 and msg.buttons[BUTTON_R1]
                     else 1.0)
            top = args.max_wheel_rad_s * args.wheel_radius
            vx = msg.axes[AXIS_LEFT_Y] * top * scale
            vy = msg.axes[AXIS_LEFT_X] * top * scale
            wz = msg.axes[AXIS_RIGHT_X] * top / args.centre_projection * scale
            vx, vy, wz, note = scale_to_usable(
                vx, vy, wz, args.wheel_radius, args.centre_projection,
                args.min_wheel_rad_s, args.max_wheel_rad_s)
            twist.twist.linear.x, twist.twist.linear.y = vx, vy
            twist.twist.angular.z = wz
            # Said on CHANGE, not per cycle: at 20 Hz the latter is a wall of
            # identical lines and the one that matters scrolls past.
            if note != state["last_note"]:
                state["last_note"] = note
                if note:
                    node.get_logger().info("stick: " + note)
            _drive_head(msg)
        # Only while L1 is held, plus one stop when it is let go (or the pad
        # is lost). Publishing a zero every cycle made this node a second
        # /cmd_vel source that fought every other one: with the pad lying on
        # the desk, a program's ri.go_velocity() alternated with these zeros
        # and the base barely moved (found on the 2026 exercise samples).
        # mecanum_drive_controller stops by itself 0.5 s after the last
        # command, so the single stop is a courtesy, not the safety.
        if driving or state["driving"]:
            twist_pub.publish(twist)
        state["driving"] = driving

    def _drive_head(msg):
        """Nudge the head while the D-pad is held.

        Published on the controller's joint_trajectory TOPIC, not sent as an
        action goal. Jogging means a new target every cycle, and twenty action
        goals a second is twenty goals each preempting the last -- accepted,
        aborted, logged, and every one of them a future nobody reads. The topic
        interface exists for exactly this and costs none of it.

        The action is still the right thing for a MOVE, which is why the
        re-sync above uses it: there the reply is the point.
        """
        if len(msg.axes) <= AXIS_DPAD_Y:
            return
        pan, tilt = msg.axes[AXIS_DPAD_X], msg.axes[AXIS_DPAD_Y]
        if pan == 0.0 and tilt == 0.0:
            # Forget the running target so the next nudge starts from where the
            # head actually IS. Holding it would let the commanded pose drift
            # away from the real one while the head is blocked or torque is
            # off, and the first press afterwards would jump it back.
            state["head_goal"] = None
            state["warned_limit"].clear()
            return
        current = state["head_goal"] or [state["measured"].get(j, 0.0)
                                         for j in head_joints]
        # L2 speeds the head up for real, unlike the base: head_rate is a
        # chosen number and not the servo's ceiling. The anti-windup below is
        # what makes asking for too much safe.
        boost = (args.head_boost
                 if len(msg.buttons) > BUTTON_L2 and msg.buttons[BUTTON_L2]
                 else 1.0)
        step = args.head_rate * boost / args.rate
        target = [current[0] + pan * step, current[1] + tilt * step]

        for i, joint in enumerate(head_joints):
            measured = state["measured"].get(joint)
            # Anti-windup first: stop ADDING once the target is far ahead of
            # where the head actually is. Whether that is a blocked joint or
            # torque that is off does not matter -- in both cases continuing to
            # integrate stores up a snap.
            if measured is not None and abs(target[i] - measured) > args.head_lead:
                target[i] = current[i]
            low, high = state["limits"].get(joint, (None, None))
            if low is not None:
                clamped = min(max(target[i], low), high)
                if clamped != target[i] and joint not in state["warned_limit"]:
                    state["warned_limit"].add(joint)
                    node.get_logger().info(
                        "{} is at its limit ({:+.3f} to {:+.3f} rad)".format(
                            joint, low, high))
                target[i] = clamped
            elif not state["limits"]:
                pass          # no description yet; the warning above covers it
        state["head_goal"] = target
        traj = JointTrajectory()
        traj.joint_names = list(head_joints)
        point = JointTrajectoryPoint()
        point.positions = target
        # Two cycles, not one: a target that is due exactly when the next one
        # arrives leaves the controller no time to reach it, and the head
        # crawls behind the stick.
        point.time_from_start = Duration(sec=0, nanosec=int(2e9 / args.rate))
        traj.points = [point]
        head_pub.publish(traj)

    node.create_timer(1.0 / args.rate, tick)
    top = args.max_wheel_rad_s * args.wheel_radius
    # The head's rate and its anti-windup are two ends of one setting, and a
    # pair that cannot work together is not something to discover by holding
    # the D-pad and watching the head crawl.
    ceiling = args.head_lead * args.rate
    if args.head_rate * args.head_boost >= ceiling:
        node.get_logger().error(
            "--head-rate {:.2f} x --head-boost {:.2f} = {:.2f} rad/s is at or "
            "past what --head-lead {:.2f} allows at {:.0f} Hz ({:.2f} rad/s). "
            "Every cycle would ask for a step the anti-windup then refuses, "
            "so L2 would make the head SLOWER. Raise --head-lead or lower "
            "one of the two.".format(
                args.head_rate, args.head_boost,
                args.head_rate * args.head_boost, args.head_lead, args.rate,
                ceiling))
    node.get_logger().info(
        "PS4 teleop ready. HOLD L1 to drive; left stick translates, right "
        "stick turns, D-pad moves the head. The base runs at its top speed "
        "{:.2f} m/s unmodified, R1 slows it to {:.2f}, and L2 does not make "
        "it faster -- there is no faster. L2 speeds the HEAD up, {:.1f} to "
        "{:.1f} rad/s. CIRCLE releases torque (no deadman needed), TRIANGLE "
        "turns it on after re-syncing the controllers. Below {:.2f} m/s the "
        "wheels do not turn at all, which is why the stick is mapped rather "
        "than proportional.".format(
            top, top * args.slow_scale, args.head_rate,
            args.head_rate * args.head_boost,
            args.min_wheel_rad_s * args.wheel_radius))
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
