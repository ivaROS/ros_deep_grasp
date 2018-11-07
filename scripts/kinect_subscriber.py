#!/usr/bin/env python
import rospy
import roslaunch
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage, Image

import time
import matplotlib.pyplot as plt
from pprint import pprint


def launch_kinect():
    package = 'freenect_launch'
    executable = 'freenect_launch'
    node = roslaunch.core.Node(package, executable)

    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    process = launch.launch(node)
    #print process.is_alive()
    process.stop()


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


if __name__ == '__main__':
    print("MAIN")
    rospy.init_node("kinect_subscriber")
#    rgb_listener()
#    depth_listener()
    while not rospy.core.is_shutdown():
        try:
            img = rospy.wait_for_message("/camera/rgb/image_raw", Image, 10)
            print(type(img))
        except Exception as e:
            print(e)


