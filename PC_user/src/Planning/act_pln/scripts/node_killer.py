#!/usr/bin/env python
import os
import rospy
from std_msgs.msg import *
from sensor_msgs.msg import *


def callback_joy(data):
    print(data.buttons[2])
    if(data.buttons[2]):
        nodes = os.popen("rosnode list").readlines()
        for i in range(len(nodes)):
            nodes[i] = nodes[i].replace("\n","")
        for node in nodes:
            os.system("rosnode kill "+ node)
        rospy.sleep(5)


def main():
    rospy.init_node('node_killer', anonymous=True)
    joy_sub = rospy.Subscriber("/joy",Joy,callback_joy)
    rospy.spin()


if __name__ == '__main__':
    main()