#!/usr/bin/python3

from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import numpy as np
import message_filters
import cv2
import numpy as np


class ControlCenter:
    def __init__(self):
        self.rate = rospy.get_param('~rate')
        self.type = rospy.get_param('~type')
        self.args = rospy.get_param('~args', [])

        self.bridge = CvBridge()

        self.subs = (message_filters.Subscriber(
            f"/cam0/image_raw/compressed",
            CompressedImage
        ), message_filters.Subscriber(
            f"/cam1/image_raw/compressed",
            CompressedImage
        ))
        
        self.ts = message_filters.ApproximateTimeSynchronizer(self.subs, 1, 1)
        self.ts.registerCallback(self.callback)
        rospy.on_shutdown(self.on_shutdown)
            
    def callback(self, *compresseds):
        self.raw_images = [
            self.bridge.compressed_imgmsg_to_cv2(compressed)
            for compressed in compresseds
        ]

        if len(self.raw_images) == 2:
            # Convert source image to unsigned 8 bit integer Numpy array
            L = cv2.cvtColor(self.raw_images[1], cv2.COLOR_BGR2GRAY)
            R = cv2.cvtColor(self.raw_images[0], cv2.COLOR_BGR2GRAY)

            stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
            disparity = stereo.compute(L, R)

            cv2.imshow('cam0', self.raw_images[0])
            cv2.imshow('cam1', self.raw_images[1])
            cv2.imshow('Depth-Map', disparity)
            cv2.waitKey(1)

    def on_shutdown(self):
        self.writer.release()

def main():
    rospy.init_node('remote_driving_node')
    node = ControlCenter()
    rospy.spin()


if __name__ == '__main__':
  main()