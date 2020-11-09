#!/usr/bin/env python

import rospy
from opencv_apps.msg import FaceArrayStamped
from std_msgs.msg import Int64

image_size = [640, 480] # pixel
image_center = list(map(lambda x: x/2, image_size))
motor_angle = 0 # [deg]

def face_detection_cb(msg):
    pub = rospy.Publisher('motor1/command', Int64, queue_size=1)

    face_msg = FaceArrayStamped()
    motor_command_msg = Int64()
    face_pos = [0, 0]
    global motor_angle

    # face check
    if len(msg.faces):
        face = msg.faces[0].face
        face_pos[0] = face.x
        face_pos[1] = face.y
        # check face position. left or right
        if face_pos[0] <= image_center[0]:
            motor_angle -= 1
        else:
            motor_angle += 1

        motor_command_msg.data = motor_angle

        # print
        print "face_pos(x, y): ({} {})".format(face_pos[0], face_pos[1])
        print "/motor1/command: {}\n".format(motor_command_msg.data)

        # publish
        pub.publish(motor_command_msg)
    else:
        print "no faces"
    
        
def main():
    rospy.init_node('motor_command_by_face', anonymous=True)
    rospy.Subscriber('face_detection/faces', FaceArrayStamped, face_detection_cb)
    rate =  rospy.Rate(10)
    
    rospy.spin()
    rate.sleep()
    
if __name__ == '__main__':
    main()

