#!/usr/bin/env python

import rospy
import pyttsx3
from std_msgs.msg import String

class speaker_ros:
    def __init__(self):
        self.engine = pyttsx3.init() 
        self.engine.setProperty('rate', 180)
        voices = self.engine.getProperty('voices')       
        self.engine.setProperty('voice', voices[2].id)
        self.sub = rospy.Subscriber("/speak", String, self.speakcall)
        self.msg_pub = rospy.Publisher("/speak", String, latch=True,queue_size=10)
        self.msg = String()
        self.msg.data = 'Hello, my name is Festino'
        self.msg_pub.publish(self.msg)

    def speakcall(self, text):
        self.engine.say(text)
        self.engine.runAndWait()
        self.engine.stop()

rospy.init_node("speaker_node")
speaker = speaker_ros()
rospy.spin()