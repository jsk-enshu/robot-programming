#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from opencv_apps.msg import FaceArrayStamped
from std_msgs.msg import Int64


class MotorCommandByFace(Node):
    def __init__(self):
        super().__init__('motor_command_by_face')

        # Image parameters
        self.image_size = [640, 480]  # pixel
        self.image_center = [x/2 for x in self.image_size]
        self.motor_angle = 0  # [deg]

        # Publisher
        self.motor_pub = self.create_publisher(Int64, 'motor1/command', 10)

        # Subscriber
        self.face_sub = self.create_subscription(
            FaceArrayStamped,
            'face_detection/faces',
            self.face_detection_cb,
            10
        )

        self.get_logger().info('Motor command by face node started')

    def face_detection_cb(self, msg):
        motor_command_msg = Int64()
        face_pos = [0, 0]

        # face check
        if len(msg.faces) > 0:
            face = msg.faces[0].face
            face_pos[0] = face.x
            face_pos[1] = face.y

            # check face position. left or right
            if face_pos[0] <= self.image_center[0]:
                self.motor_angle -= 5
            else:
                self.motor_angle += 5

            motor_command_msg.data = self.motor_angle

            # print
            self.get_logger().info(
                f"face_pos(x, y): ({face_pos[0]} {face_pos[1]})"
            )
            self.get_logger().info(
                f"/motor1/command: {motor_command_msg.data}\n"
            )

            # publish
            self.motor_pub.publish(motor_command_msg)
        else:
            self.get_logger().info("no faces")


def main(args=None):
    rclpy.init(args=args)
    node = MotorCommandByFace()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
