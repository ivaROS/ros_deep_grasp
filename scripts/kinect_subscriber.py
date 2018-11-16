#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image

import numpy as np
from PIL import Image as PILImage
from cv_bridge import CvBridge, CvBridgeError

IMAGE_TOPIC = "/camera/rgb/image_color"
DEPTH_TOPIC = "/camera/depth/image_raw"


def get_image(show=False):
    #print("CALLING GET_KINECT_IMAGE")
    rospy.init_node("kinect_subscriber")
    rgb = rospy.wait_for_message(IMAGE_TOPIC, Image)
    depth = rospy.wait_for_message(DEPTH_TOPIC, Image)

    # Convert sensor_msgs.Image readings into readable format
    bridge = CvBridge()
    rgb = bridge.imgmsg_to_cv2(rgb, rgb.encoding)
    depth = bridge.imgmsg_to_cv2(depth, depth.encoding)

    image = rgb
    image[:, :, 2] = depth
    if (show):
        im = PILImage.fromarray(image, 'RGB')
        im.show()

    return image


if __name__ == '__main__':
    image = get_image(show=True)

