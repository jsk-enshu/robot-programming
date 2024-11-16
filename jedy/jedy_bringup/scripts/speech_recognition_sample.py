#!/usr/bin/env python3

import rospy
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from geometry_msgs.msg import Twist


class VoiceCommandController:
    def __init__(self):
        rospy.init_node('voice_command_controller')

        self.linear_speed = rospy.get_param("~linear_speed", 0.2)
        self.angular_speed = rospy.get_param("~angular_speed", 0.5)

        self.cmd_vel_pub = rospy.Publisher('/diff_drive_controller/cmd_vel', Twist, queue_size=10)
        self.voice_sub = rospy.Subscriber('/speech_to_text', SpeechRecognitionCandidates, self.voice_callback)

        self.stop_msg = Twist()

        rospy.loginfo("Voice Command Controller initialized.")

    def voice_callback(self, msg):
        if not msg.transcript:
            rospy.logwarn("No transcript received.")
            return

        command = msg.transcript[0].lower()
        rospy.loginfo(f"Received command: {command}")

        if command == "進め":
            self.move_forward()
        elif command == "止まれ":
            self.stop()
        elif command == "右":
            self.turn_right()
        elif command == "左":
            self.turn_left()
        else:
            rospy.logwarn(f"Unknown command: {command}")

    def move_forward(self):
        twist = Twist()
        twist.linear.x = self.linear_speed
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo("Moving forward.")

    def stop(self):
        self.cmd_vel_pub.publish(self.stop_msg)
        rospy.loginfo("Stopping.")

    def turn_right(self):
        twist = Twist()
        twist.angular.z = -self.angular_speed
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo("Turning right.")

    def turn_left(self):
        twist = Twist()
        twist.angular.z = self.angular_speed
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo("Turning left.")


if __name__ == '__main__':
    try:
        controller = VoiceCommandController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
