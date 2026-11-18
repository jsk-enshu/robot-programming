#!/usr/bin/env python3
"""Bridge between the mechatrobot's plain-text serial protocol and ROS 2 topics.

The board runs ``firmware/uno_r4_serial``. This node gives it the same ROS 2
interface as the micro-ROS firmware (``firmware/uno_r4_microros``, env
``mechatrobot``), so the rest of mechatrobot_ros2 works with either one.

Protocol (one ASCII line per message)::

    board -> PC   range <m> | pos <deg> | led <0|1> | # <log text>
    PC -> board   cmd <deg>

Topics::

    pub  range            sensor_msgs/Range
    pub  led/state        std_msgs/Bool
    pub  motor1/position  std_msgs/Int64
    sub  motor1/command   std_msgs/Int64

Usage::

    ros2 run mechatrobot_ros2 serial_bridge.py --ros-args -p port:=/dev/ttyACM0

Threads: a reader thread only reads the port and queues lines; it wakes the
executor with a guard condition, and all publishing happens in the executor.
"""
import collections
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Range
import serial
from std_msgs.msg import Bool
from std_msgs.msg import Int64

READ_TIMEOUT_SEC = 0.1
REOPEN_INTERVAL_SEC = 1.0


class SerialBridge(Node):
    """ROS 2 node that converts serial lines to topics and back."""

    def __init__(self):
        super().__init__("mechatrobot_serial_bridge")
        self.port = self.declare_parameter("port", "/dev/ttyACM0").value
        self.baud = self.declare_parameter("baud", 115200).value
        self.frame_id = self.declare_parameter("frame_id", "range_sensor").value
        self.min_range = self.declare_parameter("min_range", 0.02).value
        self.max_range = self.declare_parameter("max_range", 4.5).value
        self.field_of_view = self.declare_parameter("field_of_view", 0.261799).value

        self.range_pub = self.create_publisher(Range, "range", 10)
        self.led_pub = self.create_publisher(Bool, "led/state", 10)
        self.position_pub = self.create_publisher(Int64, "motor1/position", 10)
        self.create_subscription(Int64, "motor1/command", self.on_command, 10)

        # (receive time, line) pairs handed from the reader thread to the executor.
        self.lines = collections.deque()
        self.lines_ready = self.create_guard_condition(self.publish_queued_lines)

        self.serial = None
        self.serial_lock = threading.Lock()
        self.running = True
        self.context.on_shutdown(self.stop_reader)
        self.reader = threading.Thread(target=self.read_loop, daemon=True)
        self.reader.start()

    # ---- PC -> board ---------------------------------------------------------

    def on_command(self, msg):
        """Send a motor target to the board.

        Parameters
        ----------
        msg : std_msgs.msg.Int64
            Target angle in degrees.
        """
        line = f"cmd {int(msg.data)}\n".encode("ascii")
        with self.serial_lock:
            if self.serial is None:
                self.get_logger().warn("not connected; motor1/command dropped")
                return
            try:
                self.serial.write(line)
            except serial.SerialException as e:
                self.get_logger().error(f"write failed: {e}")

    # ---- reader thread (no ROS calls except waking the executor) --------------

    def open_serial(self):
        """Open the port, retrying until it succeeds or the node stops."""
        while self.running:
            try:
                s = serial.Serial(self.port, self.baud, timeout=READ_TIMEOUT_SEC)
            except serial.SerialException as e:
                self.get_logger().warn(f"cannot open {self.port}: {e}", throttle_duration_sec=5.0)
                time.sleep(REOPEN_INTERVAL_SEC)
                continue
            s.reset_input_buffer()
            with self.serial_lock:
                self.serial = s
            self.get_logger().info(f"connected to {self.port} at {self.baud} bps")
            return

    def close_serial(self):
        """Close the port so that the next read reopens it."""
        with self.serial_lock:
            if self.serial is not None:
                self.serial.close()
            self.serial = None

    def read_loop(self):
        """Read lines from the board and queue them for the executor."""
        while self.running:
            if self.serial is None:
                self.open_serial()
                continue
            try:
                raw = self.serial.readline()
            except serial.SerialException as e:
                self.get_logger().error(f"read failed, reopening: {e}")
                self.close_serial()
                continue
            if not raw:
                continue
            line = raw.decode("ascii", errors="replace").strip()
            if not line or not self.running:
                continue
            self.lines.append((self.get_clock().now(), line))
            self.lines_ready.trigger()
        self.close_serial()

    def stop_reader(self):
        """Ask the reader thread to finish (called on context shutdown)."""
        self.running = False

    # ---- board -> PC (executor thread) ----------------------------------------

    def publish_queued_lines(self):
        """Publish every line the reader thread has queued."""
        while self.lines:
            received_at, line = self.lines.popleft()
            self.handle_line(received_at, line)

    def handle_line(self, received_at, line):
        """Publish one line from the board.

        Parameters
        ----------
        received_at : rclpy.time.Time
            When the line arrived on this PC; used as the message stamp.
        line : str
            A line without the trailing newline.
        """
        if line.startswith("#"):
            self.get_logger().info(f"board: {line[1:].strip()}")
            return
        key, _, value = line.partition(" ")
        try:
            if key == "range":
                self.publish_range(received_at, float(value))
            elif key == "pos":
                self.position_pub.publish(Int64(data=int(value)))
            elif key == "led":
                self.led_pub.publish(Bool(data=value.strip() == "1"))
            else:
                self.get_logger().debug(f"unknown line: {line!r}")
        except ValueError:
            # A partial line (e.g. right after the port opened) is not an error.
            self.get_logger().debug(f"malformed line: {line!r}")

    def publish_range(self, received_at, distance):
        """Publish a sensor_msgs/Range.

        Parameters
        ----------
        received_at : rclpy.time.Time
            Stamp for the message.
        distance : float
            Measured distance in meters.
        """
        msg = Range()
        msg.header.stamp = received_at.to_msg()
        msg.header.frame_id = self.frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = self.field_of_view
        msg.min_range = self.min_range
        msg.max_range = self.max_range
        msg.range = distance
        self.range_pub.publish(msg)


def main():
    """Run the bridge until interrupted."""
    rclpy.init()
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_reader()
        node.reader.join(timeout=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
