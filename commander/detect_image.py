#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def showImage(img):
    cv2.imshow('image', img)
    cv2.waitKey(1)

def process_image(msg):
    try:
        bridge = CvBridge()
        orig = bridge.imgmsg_to_cv2(msg, "bgr8")
        resized = cv2.resize(orig, None, fx=1, fy=1)
        drawImg = resized
    except Exception as err:
        print(err)
    showImage(drawImg)

def start_node():
    rospy.init_node('detect_image')
    rospy.loginfo('detect_img node started')
    rospy.Subscriber("/vehicle/camera1/image_raw", Image, process_image)
    rospy.spin()

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass