#!/usr/bin/env python3
"""Sweep the servo with the ultrasonic sensor on its horn: a slow 2D LiDAR.

The HC-SR04 is mounted on the SG90's horn. This node keeps the servo
sweeping between ``min_angle`` and ``max_angle`` and turns every distance the
board measures (20 a second) into one point at the angle it was measured at.
rviz keeps old points for its decay time, so the picture fills in over time.

The step per command (``step_deg``, 7.3 deg by default) is not a divisor of
the sweep, so each pass measures at angles between those of the passes before
it: like a Livox, the longer you wait, the denser it gets.

Pairing a distance with its angle: in every 50 ms tick the firmware measures
the distance, then sends ``range`` and right after it ``pos`` with the angle
it had written to the servo while measuring. This node waits for that ``pos``
and does not use the one before it, which is 50 ms (up to 10 deg) older.

Topics::

    sub  range                sensor_msgs/Range (from serial_bridge.py)
    sub  motor1/position      std_msgs/Int64 [deg]
    pub  motor1/command       std_msgs/Int64 [deg]
    pub  servo_scan/points    sensor_msgs/PointCloud2, one point per distance
                              (x, y, z, intensity = distance [m])
    pub  servo_scan/beam      visualization_msgs/Marker, the current beam

Usage (with serial_bridge.py running, and nothing else commanding motor1)::

    ros2 launch mechatrobot_ros2 servo_scan.launch.py

The servo stays where it was when the node stops.
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from sensor_msgs.msg import Range
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Int64
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
]


class ServoScan(Node):
    """ROS 2 node that sweeps the servo and publishes what the sensor sees."""

    def __init__(self):
        super().__init__("servo_scan")
        self.min_angle = self.number("min_angle", -90.0)
        self.max_angle = self.number("max_angle", 90.0)
        # Not a divisor of the sweep, so the angles shift from pass to pass.
        self.step_deg = self.number("step_deg", 7.3)
        # Seconds between commands; the board sends a distance every 0.05 s.
        period = self.number("command_period", 0.05)
        # +1 when a positive servo angle turns the sensor counter-clockwise
        # seen from above (the x-y plane of frame_id), -1 when clockwise.
        self.direction = self.number("direction", 1.0)
        # From the servo axis to the front of the sensor [m].
        self.sensor_offset = self.number("sensor_offset", 0.02)
        self.frame_id = self.declare_parameter("frame_id", "servo_scan").value

        self.command_angle = 0.0
        self.sweep_sign = 1.0
        self.pending_range = None  # Range waiting for the pos of the same tick

        self.command_pub = self.create_publisher(Int64, "motor1/command", 10)
        self.points_pub = self.create_publisher(PointCloud2, "servo_scan/points", 10)
        self.beam_pub = self.create_publisher(Marker, "servo_scan/beam", 10)
        self.create_subscription(Range, "range", self.on_range, 10)
        self.create_subscription(Int64, "motor1/position", self.on_position, 10)
        self.create_timer(period, self.on_timer)
        self.get_logger().info(
            f"sweeping {self.min_angle:.0f}..{self.max_angle:.0f} deg, "
            f"{self.step_deg} deg every {period} s; points on servo_scan/points")

    def number(self, name, default):
        """Declare a numeric parameter that accepts both 1 and 1.0.

        Parameters
        ----------
        name : str
            Parameter name.
        default : float
            Default value.

        Returns
        -------
        float
            The parameter's value.
        """
        descriptor = ParameterDescriptor(dynamic_typing=True)
        return float(self.declare_parameter(name, default, descriptor).value)

    def on_timer(self):
        """Advance the commanded angle, turning back at either end."""
        angle = self.command_angle + self.sweep_sign * self.step_deg
        if angle > self.max_angle:
            angle = 2 * self.max_angle - angle
            self.sweep_sign = -1.0
        elif angle < self.min_angle:
            angle = 2 * self.min_angle - angle
            self.sweep_sign = 1.0
        self.command_angle = angle
        self.command_pub.publish(Int64(data=int(round(angle))))

    def on_range(self, msg):
        """Keep a distance until the angle of the same tick arrives.

        Parameters
        ----------
        msg : sensor_msgs.msg.Range
            Distance from the board.
        """
        self.pending_range = msg

    def on_position(self, msg):
        """Turn the waiting distance and this angle into a point.

        Parameters
        ----------
        msg : std_msgs.msg.Int64
            Angle the firmware wrote to the servo [deg], sent right after the
            distance measured in the same tick.
        """
        rng = self.pending_range
        self.pending_range = None
        if rng is None:
            return
        theta = math.radians(self.direction * msg.data)
        echo = rng.min_range < rng.range < rng.max_range
        # No echo (0) is drawn as a short beam, not as a point.
        distance = rng.range + self.sensor_offset if echo else self.sensor_offset + 0.1
        x = distance * math.cos(theta)
        y = distance * math.sin(theta)
        if echo:
            cloud = point_cloud2.create_cloud(rng.header, FIELDS, [(x, y, 0.0, rng.range)])
            cloud.header.frame_id = self.frame_id
            self.points_pub.publish(cloud)
        self.publish_beam(rng.header.stamp, x, y, echo)

    def publish_beam(self, stamp, x, y, echo):
        """Draw the current beam from the servo axis to the measured point.

        Parameters
        ----------
        stamp : builtin_interfaces.msg.Time
            Stamp of the distance.
        x, y : float
            End of the beam [m].
        echo : bool
            Whether the sensor got an echo (red line when not).
        """
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.ns = "beam"
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.005
        marker.points = [Point(), Point(x=x, y=y)]
        marker.color.a = 1.0
        if echo:
            marker.color.g = 1.0
        else:
            marker.color.r = 1.0
        self.beam_pub.publish(marker)


def main():
    """Run the node until interrupted."""
    rclpy.init()
    node = ServoScan()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
