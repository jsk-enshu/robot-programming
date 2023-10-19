#!/usr/bin/env python3
# coding=utf-8
import rospy
from geometry_msgs.msg import *
from speech_recognition_msgs.msg import *
from speech_recognition_msgs.srv import *

class sample_julius_cmd:

    def __init__(self):
        rospy.init_node("sample_julius_cmd")
        rospy.Subscriber("/speech_to_text", SpeechRecognitionCandidates, self.cb)
        self.pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=1)

        rospy.loginfo("Setting up vocabulary...")
        rospy.wait_for_service('/speech_recognition')
        try:
            change_vocabulary = rospy.ServiceProxy('/speech_recognition', SpeechRecognition)
            req = SpeechRecognitionRequest()
            req.vocabulary.words = ["みぎ","ひだり","まえ","うしろ","だんす"]
            req.quiet = True
            res = change_vocabulary(req)
            print res
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        rospy.loginfo("OK")
        rospy.spin()

    def cb(self, msg):
        rospy.loginfo(msg.transcript[0])
        vel = Twist()

        if msg.transcript[0] == "まえ":
            vel.linear.x = 0.2
        
        if msg.transcript[0] == "うしろ":
            vel.linear.x = -0.2
        
        if msg.transcript[0] == "みぎ":
            vel.angular.z = -1
        
        if msg.transcript[0] == "ひだり":
            vel.angular.z = 1

        self.pub.publish(vel)

        
if __name__ == '__main__':
    instance = sample_julius_cmd()
