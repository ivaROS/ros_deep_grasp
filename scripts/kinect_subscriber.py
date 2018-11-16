#!/usr/bin/env python
import rospy
import roslaunch
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage, Image

import time
import matplotlib.pyplot as plt
from pprint import pprint
import numpy as np
from PIL import Image as PILImage
import ros_numpy
import cv2
from cv_bridge import CvBridge, CvBridgeError


#def launch_kinect():
#    package = 'freenect_launch'
#    executable = 'freenect_launch'
#    node = roslaunch.core.Node(package, executable)
#
#    launch = roslaunch.scriptapi.ROSLaunch()
#    launch.start()
#
#    process = launch.launch(node)
#    #print process.is_alive()
#    process.stop()


#def rgb_callback(data):
#    print("RGB_CALLBACK")
#    print(data._connection_header)
#    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
#    #im = data
#    #fig, ax = plt.subplots(figsize=(12, 12))
#    #ax.imshow(im, aspect='equal')
#    for i in range(0, len(data.data)):
#        print(data.data[i])


#def depth_callback(data):
#    print("DEPTH CALLBACK")


#def rgb_listener():
#    print("KINECT SUBSCRIBER RGB LISTENER")
#    #rospy.init_node('rgb_listener', anonymous=True)
#    sub = rospy.Subscriber("/camera/rgb/image_raw", Image, rgb_callback) #, queue_size=10)
#    print(sub)
#    #print(sub.get_num_connections())
#    #print(sub.name)
#    #print(sub.data_class)
#    #print(sub.callback)
#    pprint(vars(sub))
#    #print(rospy.core)
#    #rospy.spin()
#    #while not rospy.core.is_shutdown():
#        #rospy.rostime.wallsleep(0.5)
#    while True:
#        stop = rospy.core.is_shutdown()
#        print(stop)
#        if (stop):
#            return
#        time.sleep(0.5)


#def depth_listener():
#    print("KINECT SUBSCRIBER DEPTH LISTENER")
#    #rospy.init_node('depth_listener', anonymous=True)
#    rospy.Subscriber("/camera/depth/image_raw", Image, depth_callback)
#    rospy.spin()


def get_image(show=False):
    print("CALLING GET_KINECT_IMAGE")
    rospy.init_node("kinect_subscriber")
    image = rospy.wait_for_message("/camera/rgb/image_color", Image)

    # Convert sensor_msgs.Image readings into readable format
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(image, image.encoding)

    if (show):
        im = PILImage.fromarray(image, 'RGB')
        im.show()

    return image


if __name__ == '__main__':
    image = get_image()


#if __name__ == '__main__':
#    print("MAIN")
#    rospy.init_node("kinect_subscriber")
##    rgb_listener()
##    depth_listener()
#    count = 0
#    r = []
#    g = []
#    b = []
#    while not rospy.core.is_shutdown():
#        try:
#            img = rospy.wait_for_message("/camera/rgb/image_raw", Image, 10)
#            #print(img.height, img.width)
#            #print(img)
#
#            if (count % 3 == 0):
#                r = img
#            elif (count % 3 == 1):
#                g = img
#            else:
#                b = img
#                rgb_img = [r, g, b]
#                #print(rgb_img)
#
#        except Exception as e:
#            print(e)
#        count += 1


