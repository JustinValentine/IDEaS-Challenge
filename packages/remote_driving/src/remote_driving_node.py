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
        self.raw_images = None

        self.subs = rospy.Subscriber(f"/cam0/image_raw/compressed", CompressedImage, self.callback)
        

    def callback(cam0_image_compressed):
        # Convert compressed image to OpenCV format
        np_arr = np.fromstring(cam0_image_compressed.data, np.uint8)
        cam0_image_cv = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Display the image
        cv2.imshow('cam0_image', cam0_image_cv)
        cv2.waitKey(1)


    def run(self):
        rate = rospy.Rate(self.rate)

        while not rospy.is_shutdown():
            rate.sleep()
            if self.raw_images is None:
                continue
            state = self.perception.get_state(self.raw_images)
            self.publish_state(state)
            

def main():
    rospy.init_node('remote_driving_node')
    node = ControlCenter()
    node.run()
    rospy.spin()


if __name__ == '__main__':
  main()