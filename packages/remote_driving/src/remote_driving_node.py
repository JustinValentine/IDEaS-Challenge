#!/usr/bin/python3

from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import numpy as np
import message_filters
import cv2


class ControlCenter:
    def __init__(self):
        self.rate = rospy.get_param('~rate')
        self.type = rospy.get_param('~type')
        self.args = rospy.get_param('~args', [])

        self.bridge = CvBridge()

        self.subs = rospy.Subscriber(f"/cam0/image_raw/compressed", CompressedImage, self.callback)
        

    def callback(self, compressed):
        # Convert compressed image to OpenCV format
        cam0_image_cv = self.bridge.compressed_imgmsg_to_cv2(compressed)

        # Display the image
        cv2.imshow('cam0_image', cam0_image_cv)
        cv2.waitKey(1)
            

def main():
    rospy.init_node('remote_driving_node')
    node = ControlCenter()
    rospy.spin()


if __name__ == '__main__':
  main()