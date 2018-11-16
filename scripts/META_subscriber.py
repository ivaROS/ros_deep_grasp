#!/usr/bin/env python

import sys, time
import roslib
import rospy
from sensor_msgs.msg import Image

def callback(data):
    print("Got an image I think")

def META_listener():
    rospy.init_node('META_listener', anonymous=False)
    rospy.Subscriber("camera/image")

    rospy.spin()

if __name__ == '__main__':
    META_listener()
