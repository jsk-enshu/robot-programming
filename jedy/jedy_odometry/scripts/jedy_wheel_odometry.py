#!/usr/bin/env python3
"""Publish wheel odometry for Jedy's mecanum base, and say how much to trust it.

    ros2 run jedy_odometry jedy_wheel_odometry

Reads the raw potentiometer pulses the driver publishes on
``/actuo_bus_servicer/servo/state`` (channel ``pulse``), turns each into a
continuous wheel angle with :class:`~jedy_odometry.wheel_angle.WheelAngle`, and
integrates the four into a base pose.

Why this exists as a separate node
----------------------------------
``mecanum_drive_controller`` computes odometry from a wheel VELOCITY state
interface, and this base has none to give: the driver publishes NaN there
deliberately, and the controller correctly skips its integration rather than
believing a number. That is the right behaviour and it is not going to change
-- the driver must not publish an estimate as a measurement. So the estimate
lives here instead, on its own topic, built from the raw channel.

Two refusals that are the point
-------------------------------
* **It publishes into ``odom`` only if asked** (``--publish-tf``, default off).
  A frame claiming to know where the robot is gets believed by everything
  downstream, and this estimate has a wheel-slip problem it cannot see (below).
* **It publishes its own quality alongside the pose** on ``~/scale_spread``:
  how much each wheel's learned revolutions disagree with each other. That is
  the number that matters, because the revolution COUNT cannot drift -- it
  counts wraps -- so every uncertainty in a distance lives in the learned
  scale. Measured on the robot: 1.1 to 3.3 % across the four wheels. A pose
  published without it is a claim nobody can check.

The honest limit, which is not the encoder
------------------------------------------
The potentiometer is good. Replayed against a real 8 s trace it gives the
revolution count exactly and a speed matching an independent 1/period to 0.2 %,
with the scale it taught itself agreeing to 1-3 %. Mecanum ROLLERS are the
problem. They slip by design -- that is how the base strafes -- so wheel
rotation is a weaker statement about where the robot went than it would be on a
differential base, and it is worst in exactly the motions this base exists for.
Treat this as a velocity estimate and a short-horizon dead reckoning, and fuse
it with something that sees the world before believing a position from it.
"""

from __future__ import annotations

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (  # noqa: E402
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy)

from geometry_msgs.msg import TransformStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from std_msgs.msg import Float64  # noqa: E402
from actuo_msgs.msg import ServoState  # noqa: E402

from jedy_odometry.wheel_angle import WheelAngle  # noqa: E402

#: Corner order the mecanum forward kinematics below is written in.
#: Derived, not read off the joint origins: dummy_joint_rleg carries
#: rpy="pi 0 0", so mechanum_base_link is rolled 180 degrees against base_link
#: and every wheel's y flips. See jedy_four_dof_kondo.yaml, which must agree.
CORNERS = {
    "front_left": "mechanum_joint2",
    "front_right": "mechanum_joint1",
    "rear_left": "mechanum_joint4",
    "rear_right": "mechanum_joint3",
}

#: Must match jedy_four_dof_controllers.yaml. Duplicated deliberately rather
#: than read from the controller: this node runs without the controller, and a
#: value that silently differs would produce a pose that looks fine and drifts.
#: The launch passes them, so the two are set in one place in practice.
WHEEL_RADIUS = 0.03
CENTRE_PROJECTION = 0.11975          # |x| + |y| of a wheel from the base centre


def parse_args(argv):
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--servo-state", default="/actuo_bus_servicer/servo/state")
    p.add_argument("--wheel-radius", type=float, default=WHEEL_RADIUS)
    p.add_argument("--centre-projection", type=float, default=CENTRE_PROJECTION,
                   help="|x| + |y| of a wheel from the base centre, the same "
                        "number mecanum_drive_controller calls "
                        "sum_of_robot_center_projection_on_X_Y_axis")
    p.add_argument("--directions", nargs=4, type=int,
                   default=[1, -1, 1, 1], metavar=("J1", "J2", "J3", "J4"),
                   help="per-joint direction, in mechanum_joint1..4 order, "
                        "matching the robot config. joint4 is +1 on Jedy "
                        "because its URDF axis is flipped relative to the "
                        "other three -- see the config's note before changing.")
    p.add_argument("--frame", default="odom")
    p.add_argument("--child-frame", default="base_link")
    p.add_argument("--publish-tf", action="store_true",
                   help="also broadcast odom->base_link. OFF by default: a "
                        "frame claiming to know where the robot is gets "
                        "believed by everything downstream, and mecanum "
                        "rollers slip in exactly the motions this base is for.")
    p.add_argument("--ros-args", dest="ros_args", nargs=argparse.REMAINDER,
                   default=[])
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    rclpy.init(args=args.ros_args or None)
    node = Node("jedy_wheel_odometry")

    directions = dict(zip(
        ["mechanum_joint1", "mechanum_joint2", "mechanum_joint3",
         "mechanum_joint4"], args.directions))
    wheels = {name: WheelAngle(direction=directions[name])
              for name in CORNERS.values()}
    previous = {name: None for name in CORNERS.values()}

    odom_pub = node.create_publisher(Odometry, "~/odometry", 10)
    quality_pub = node.create_publisher(Float64, "~/scale_spread", 10)
    tf_pub = None
    if args.publish_tf:
        from tf2_ros import TransformBroadcaster
        tf_pub = TransformBroadcaster(node)
        node.get_logger().warn(
            "publishing odom->{} TF. This pose comes from wheel rotation only "
            "and mecanum rollers slip; nothing downstream can tell.".format(
                args.child_frame))

    pose = {"x": 0.0, "y": 0.0, "theta": 0.0}
    state = {"last_stamp": None}

    qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def on_state(msg):
        stamp = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        angles = {}
        for joint in msg.joints:
            if joint.name in wheels:
                aux = dict(zip(joint.aux_names, joint.aux_values))
                if "pulse" in aux:
                    angles[joint.name] = wheels[joint.name].update(
                        aux["pulse"], stamp)
        if len(angles) < 4:
            return

        if state["last_stamp"] is None:
            state["last_stamp"] = stamp
            for name in wheels:
                previous[name] = angles[name]
            return
        dt = stamp - state["last_stamp"]
        if dt <= 0.0:
            return
        state["last_stamp"] = stamp

        # POSE from the angle, TWIST from the wrap period, and the split is
        # not fussiness. Differentiating the angle per message over-reads by
        # about 1.44x on this hardware: the reading advances in steps, so the
        # instantaneous rate is spiky and biased high even though the SUM over
        # time is right. Measured against a commanded 0.796 rev/s: the wrap
        # period gave 0.795, 0.783, 0.795, 0.789 -- and differentiating the
        # angle gave 1.148, 0.758, 1.150, 1.137.
        #
        # So the pose integrates the angle, where the spikes average out and
        # the total is exact, and the twist reports the wrap-period speed,
        # which is accurate to 0.2 % but a revolution behind. Both are honest
        # about what they are; using one number for both would not be.
        pose_rates, twist_rates = {}, {}
        for name, wheel in wheels.items():
            pose_rates[name] = (angles[name] - previous[name]) / dt
            previous[name] = angles[name]
            speed = wheel.speed_rev_s
            twist_rates[name] = (0.0 if speed is None
                                 else speed * 2.0 * math.pi * wheel.direction)

        # Mecanum forward kinematics, the inverse of what
        # mecanum_drive_controller applies. Written out rather than imported
        # because the controller does not expose it, and its own version is
        # unusable here for the reason in the module docstring.
        r = args.wheel_radius
        k = args.centre_projection

        def body_twist(rates):
            fl = rates[CORNERS["front_left"]]
            fr = rates[CORNERS["front_right"]]
            rl = rates[CORNERS["rear_left"]]
            rr = rates[CORNERS["rear_right"]]
            return (r * (fl + fr + rl + rr) / 4.0,
                    r * (-fl + fr + rl - rr) / 4.0,
                    r * (-fl + fr - rl + rr) / (4.0 * k))

        step_vx, step_vy, step_wz = body_twist(pose_rates)
        vx, vy, wz = body_twist(twist_rates)

        pose["theta"] += step_wz * dt
        pose["x"] += (step_vx * math.cos(pose["theta"])
                      - step_vy * math.sin(pose["theta"])) * dt
        pose["y"] += (step_vx * math.sin(pose["theta"])
                      + step_vy * math.cos(pose["theta"])) * dt

        odom = Odometry()
        odom.header.stamp = msg.stamp
        odom.header.frame_id = args.frame
        odom.child_frame_id = args.child_frame
        odom.pose.pose.position.x = pose["x"]
        odom.pose.pose.position.y = pose["y"]
        odom.pose.pose.orientation.z = math.sin(pose["theta"] / 2.0)
        odom.pose.pose.orientation.w = math.cos(pose["theta"] / 2.0)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        # The covariance is NOT filled in. It would have to be invented: the
        # dominant error here is roller slip, which nothing in this node can
        # see, and a plausible number in that field is what makes a filter
        # believe this pose more than it should. Left at zero, which ROS reads
        # as "unknown".
        odom_pub.publish(odom)

        spreads = [w.scale_spread for w in wheels.values()
                   if w.scale_spread is not None]
        worst = max(spreads) if spreads else float("nan")
        quality_pub.publish(Float64(data=float(worst)))

        if tf_pub is not None:
            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = args.child_frame
            transform.transform.translation.x = pose["x"]
            transform.transform.translation.y = pose["y"]
            transform.transform.rotation = odom.pose.pose.orientation
            tf_pub.sendTransform(transform)

    node.create_subscription(ServoState, args.servo_state, on_state, qos)
    node.get_logger().info(
        "odometry from wheel rotation only. Each wheel learns its own pulses "
        "per revolution from its wraps, so no scale is assumed; ~/scale_spread "
        "reports how much those disagree, which is where the distance "
        "uncertainty lives. Roller slip is not in that number and is invisible "
        "from here. Publishing ~/odometry and ~/scale_spread{}.".format(
            "" if tf_pub is None else " and odom TF"))
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
