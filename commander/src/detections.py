#!/usr/bin/env python3


import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

import message_filters

# hsv ranges for green
lb1 = np.array([55, 127, 36])
ub1 = np.array([88, 255, 255])

#hsv ranges for orange
lb2 = np.array([0, 100, 100])
ub2 = np.array([20, 255, 255])

def process_image(img_x, img_y):
    try:
        bridge = CvBridge()
        img_x = bridge.imgmsg_to_cv2(img_x,"bgr8")
        img_y = bridge.imgmsg_to_cv2(img_y,"bgr8")
        # cv_image = cv2.resize(cv_image, (800,800))
        # cv_image = np.uint8(cv_image)

        hsv_x = cv2.cvtColor(img_x, cv2.COLOR_BGR2HSV)
        mask_g = cv2.inRange(hsv_x, lb1, ub1) # green mask
        mask_o = cv2.inRange(hsv_x, lb2, ub2) # orange mask

        res = cv2.bitwise_and(img_x, img_x, mask = mask_g)
        gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        _, thresh = cv2.threshold(blur, 20, 255, cv2.THRESH_BINARY)
        dilated = cv2.dilate(thresh, None, iterations=3)
        contours_x_green, _ = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        res = cv2.bitwise_and(img_x, img_x, mask = mask_o)
        gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        _, thresh = cv2.threshold(blur, 20, 255, cv2.THRESH_BINARY)
        dilated = cv2.dilate(thresh, None, iterations=3)
        contours_x_orange, _ = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        dataset = Float32MultiArray()
        my_data = [-1 for i in range(8)]

        for contour in contours_x_orange[::-1]: # robo
            x, y, w, h = cv2.boundingRect(contour)
            # print("Detecting robo")
            if cv2.contourArea(contour) < 500:
                continue
            my_data[0] = x+w/2
            my_data[1] = y+h/2
            # print(w)
            cv2.rectangle(img_x, (x,y), (x+w, y+h), (0,0,255), 2)

        for contour in contours_x_green: # green ball
            x, y, w, h = cv2.boundingRect(contour)
            # print(str(x) + " " + str(y))
            if cv2.contourArea(contour) < 200:
                continue
            my_data[2] = x+w/2
            my_data[3] = y+h/2
        
        res = cv2.bitwise_and(img_y, img_y, mask = mask_g)
        gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        _, thresh = cv2.threshold(blur, 20, 255, cv2.THRESH_BINARY)
        dilated = cv2.dilate(thresh, None, iterations=3)
        contours_x_green, _ = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        res = cv2.bitwise_and(img_y, img_y, mask = mask_o)
        gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        _, thresh = cv2.threshold(blur, 20, 255, cv2.THRESH_BINARY)
        dilated = cv2.dilate(thresh, None, iterations=3)
        contours_x_orange, _ = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours_x_orange[::-1]: # robo
            x, y, w, h = cv2.boundingRect(contour)
            # print("Detecting robo")
            if cv2.contourArea(contour) < 500:
                continue
            my_data[4] = x+w/2
            my_data[5] = y+h/2
            # print(w)
            cv2.rectangle(img_y, (x,y), (x+w, y+h), (0,0,255), 2)

        for contour in contours_x_green: # green ball
            x, y, w, h = cv2.boundingRect(contour)
            # print(str(x) + " " + str(y))
            if cv2.contourArea(contour) < 200:
                continue
            my_data[6] = x+w/2
            my_data[7] = y+h/2

        dataset.data = my_data
        pub.publish(dataset)

    except Exception as err:
        print(err)

def start_node():
    global pub
    rospy.init_node('detections', anonymous=True)
    rospy.loginfo('detections node started')
    
    img_x = message_filters.Subscriber("/vehicle/camera_x/image", Image)
    img_y = message_filters.Subscriber("/vehicle/camera_y/image", Image)
    ts = message_filters.TimeSynchronizer([img_x, img_y], 10)
    ts.registerCallback(process_image)
    pub = rospy.Publisher('/dataset', Float32MultiArray, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass