#!/usr/bin/env python3

from cv2 import data
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

def command(dataset):
    try:
        # suffix 1 refers to camera1 input and suffix 2 refers to camera 2 inputs
        robo_x1, robo_y1, obj_x1, obj_y1, robo_x2, robo_y2, obj_x2, obj_y2 = dataset.data
        print(data.data)
        
    except Exception as err:
        print(err)

def start_node():
    global pub
    rospy.init_node('command', anonymous=True)
    rospy.loginfo('command node started')
    rospy.Subscriber("/dataset", Float32MultiArray, command)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass