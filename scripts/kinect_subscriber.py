#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image

from PIL import Image as PILImage
from cv_bridge import CvBridge, CvBridgeError


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
    image = get_image(show=True)

