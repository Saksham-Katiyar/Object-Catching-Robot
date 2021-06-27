#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

frame1 = np.ndarray((800, 800, 3), dtype=np.uint8)

def showImage(img):
    cv2.imshow('image', img)
    cv2.waitKey(1)

def publish(x1, x2):
    vel_msg = Twist()
    v0 = 2
    dx = x1-x2
    if dx>10:
        vel_x = v0
    elif dx<10:
        vel_x = -v0
    else:
        vel_x = 0
    if (x1 == -1) or (x2 == -1):
        vel_x = 0
    vel_msg.linear.x = vel_x
    # vel_msg.linear.z = 100
    pub.publish(vel_msg)

def nothing(x):
    pass

def process_image(msg):
    global frame1
    try:
        bridge = CvBridge()
        frame2 = bridge.imgmsg_to_cv2(msg, "bgr8")

        # l_h = cv2.getTrackbarPos("LH", "Tracking")
        # l_s = cv2.getTrackbarPos("LS", "Tracking")
        # l_v = cv2.getTrackbarPos("LV", "Tracking")
        
        # u_h = cv2.getTrackbarPos("UH", "Tracking")
        # u_s = cv2.getTrackbarPos("US", "Tracking")
        # u_v = cv2.getTrackbarPos("UV", "Tracking")
        
        # lb1 = np.array([l_h, l_s, l_v])
        # ub1 = np.array([u_h, u_s, u_v])

        lb1 = np.array([55, 127, 36]) # for green
        ub1 = np.array([88, 255, 255])
        
        hsv1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2HSV)
        hsv2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2HSV)
        
        mask = cv2.inRange(hsv1, lb1, ub1)
        
        res1 = cv2.bitwise_and(frame1, frame1, mask = mask)
        res2 = cv2.bitwise_and(frame2, frame2, mask = mask)
        
        # cv2.imshow("mask", mask)
        # cv2.imshow("result", res2)

        diff = cv2.absdiff(res1, res2)
        gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        _, thresh = cv2.threshold(blur, 20, 255, cv2.THRESH_BINARY)
        dilated = cv2.dilate(thresh, None, iterations=3)
        contours, _ = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        lb2 = np.array([0, 100, 100]) # for orange
        ub2 = np.array([20, 255, 255])

        mask_2 = cv2.inRange(hsv1, lb2, ub2)

        res = cv2.bitwise_and(frame1, frame1, mask = mask_2)
        gray2 = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
        blur2 = cv2.GaussianBlur(gray2, (5,5), 0)
        _, thresh2 = cv2.threshold(blur2, 20, 255, cv2.THRESH_BINARY)
        dilated2 = cv2.dilate(thresh2, None, iterations=3)
        contours2, _ = cv2.findContours(dilated2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        x1, x2 = -1, -1

        for contour in contours2: # robo
            x, y, w, h = cv2.boundingRect(contour)
            # print("Detecting robo")
            if cv2.contourArea(contour) < 100:
                continue
            x2 = x+w/2
            cv2.rectangle(frame1, (x,y), (x+w, y+h), (0,0,255), 2)

        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            
            if cv2.contourArea(contour) < 700:
                continue
            x1 = x+w/2
            cv2.rectangle(frame1, (x,y), (x+w, y+h), (0,0,255), 2)
            # xar.append(int(len(yar)+1))
            # yar.append(int(x+w/2))
            
            cv2.putText(frame1, "Status: {}".format('Movement'), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 3)
        #cv2.drawContours(frame1, contours, -1, (255, 0, 0), 2)
        
        publish(x1, x2)

        # cv2.imshow("feed", frame1)
        
        # ret, frame2 = cap.read()
        
        # if (cv2.waitKey(1) & 0xFF == ord('q')):
        #     break

        drawImg = frame1
        showImage(drawImg)
        frame1 = frame2
        
    except Exception as err:
        print(err)

def start_node():
    global pub
    rospy.init_node('detect_image')
    rospy.loginfo('detect_img node started')
    rospy.Subscriber("/vehicle/camera1/image", Image, process_image)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    try:
        # cv2.namedWindow("Tracking")
        # cv2.createTrackbar("LH", "Tracking", 55, 255, nothing)
        # cv2.createTrackbar("LS", "Tracking", 127, 255, nothing)
        # cv2.createTrackbar("LV", "Tracking", 36, 255, nothing)
        # cv2.createTrackbar("UH", "Tracking", 88, 255, nothing)
        # cv2.createTrackbar("US", "Tracking", 255, 255, nothing)
        # cv2.createTrackbar("UV", "Tracking", 255, 255, nothing)
        start_node()
    except rospy.ROSInterruptException:
        pass