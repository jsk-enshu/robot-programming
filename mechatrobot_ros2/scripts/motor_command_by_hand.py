#!/usr/bin/env python3
"""Turn the mechatrobot's stepper towards the side of the image your hand is on.

Reads the hands that jedy_vision's ``hand_detection_node`` finds in the camera
image and sends an absolute angle to ``motor1/command``:

    hand in the right part of the image   angle += step_deg
    hand in the left part of the image    angle -= step_deg
    hand near the middle, or no hand      no command (the motor stays)

``hand_detection_node`` publishes once for every image it reads (about 10 a
second), so the default step (3 deg) asks for about 30 deg/s, the 28BYJ-48's
speed at 5 rpm.

Topics::

    sub  /hand_detection/hands  jedy_vision_msgs/DetectionArray
    pub  motor1/command         std_msgs/Int64 [deg]
"""
from jedy_vision_msgs.msg import DetectionArray
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Int64


class MotorCommandByHand(Node):
    """ROS 2 node that turns the stepper towards the hand."""

    def __init__(self):
        super().__init__('motor_command_by_hand')
        self.step_deg = self.declare_parameter('step_deg', 3).value
        # usb_cam's default image size [pixel]
        self.image_width = self.declare_parameter('image_width', 640).value
        # No command while the hand is within this fraction of the width around
        # the centre, so a hand held still does not make the motor hunt.
        self.dead_zone = self.declare_parameter('dead_zone', 0.2).value
        self.motor_angle = 0  # [deg]
        self.motor_pub = self.create_publisher(Int64, 'motor1/command', 10)
        self.create_subscription(DetectionArray, '/hand_detection/hands', self.on_hands, 10)
        self.get_logger().info('move your hand to the right or left of the image')

    def on_hands(self, msg):
        """Send a new angle when a hand is right or left of the centre.

        Parameters
        ----------
        msg : jedy_vision_msgs.msg.DetectionArray
            Hands found in one image; ``box`` is x1, y1, x2, y2 [pixel].
        """
        if len(msg.detections) == 0:
            return
        # The most confident hand
        hand = max(msg.detections, key=lambda d: d.score)
        x1, _, x2, _ = hand.box
        offset = (x1 + x2) / 2 / self.image_width - 0.5  # -0.5 (left) .. 0.5 (right)
        if abs(offset) < self.dead_zone / 2:
            return
        self.motor_angle += self.step_deg if offset > 0 else -self.step_deg
        self.motor_pub.publish(Int64(data=self.motor_angle))
        self.get_logger().info(
            f'{hand.label} at x={(x1 + x2) / 2:.0f} px -> motor1/command {self.motor_angle}',
            throttle_duration_sec=1.0)


def main():
    """Run the node until interrupted."""
    rclpy.init()
    node = MotorCommandByHand()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
