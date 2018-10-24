#!/usr/bin/env python
import rospy
import roslaunch
from std_msgs.msg import String


def launch_kinect():
    package = 'freenect_launch'
    executable = 'freenect_launch'
    node = roslaunch.core.Node(package, executable)

    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    
    process = launch.launch(node)
    print process.is_alive()
    process.stop()


def callback(data):
    print("CALLBACK")
    print(data._connection_header)
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def rgb_listener():
    print("KINECT SUBSCRIBER RGB LISTENER")
    rospy.init_node('rgb_listener', anonymous=True)
    rospy.Subscriber("/camera/rgb/image_raw", String, callback, queue_size=10)
    #rospy.Subscriber("/camera/rgb/camera_info", String, callback)
    #rospy.Subscriber("/camera/sensor_msgs/CameraInfo", String, callback)
    rospy.spin()


def depth_listener():
    print("KINECT SUBSCRIBER DEPTH LISTENER")
    rospy.init_node('depth_listener', anonymous=True)
    rospy.Subscriber("/camera/depth/image_raw", String, callback)
    rospy.spin()


if __name__ == '__main__':
    print("KINECT SUBSCRIBER")
    # launch_kinect()
    rgb_listener()
    #depth_listener()


