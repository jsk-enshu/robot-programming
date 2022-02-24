#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

import sys, select, termios, tty
def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

move = {
    'w' : lambda : (1,0,0),
    'a' : lambda : (0,1,0),
    's' : lambda : (-1,0,0),
    'd' : lambda : (0,-1,0),
    'q' : lambda : (0,0,1),
    'e' : lambda : (0,0,-1),
}

if __name__ == '__main__':
    print """
Reading from keyboard
----------------------------
User 'wsad' to translate
User 'qe' to yaw
Press 'Esc' to quit
"""
settings = termios.tcgetattr(sys.stdin)

try:
    rospy.init_node('send_velocity', anonymous=True)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=100);
    r = rospy.Rate(19) # 10hz
    while not rospy.is_shutdown() :
        key = getKey()
        print key
        if key in ['\x1b']:
            break
        elif key in move.keys():
            print key,move[key]()
            cmd = Twist()
            cmd.linear.x = move[key]()[0]
            cmd.linear.y = move[key]()[1]
            cmd.angular.z = move[key]()[2]
            print "publish ",cmd
            pub.publish(cmd)
        r.sleep()
except rospy.ROSInterruptException: pass
