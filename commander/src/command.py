#!/usr/bin/env python3

import warnings
import rospy
import cv2
import numpy as np
import time
import math
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist 
from cv_bridge import CvBridge

vel_msg = Twist()

prev_time=time.time()
x_object_prev=0
y_object_prev=0
start_x_object=0
i=0
j=0

lb1 = np.array([55, 127, 36]) # for green
ub1 = np.array([88, 255, 255])

lb2 = np.array([0, 100, 100]) # for orange
ub2 = np.array([20, 255, 255])


# bl2,gl2,rl2,bu2,gu2,ru2=0,63,132,12,73,145

# l_bot=np.array([bl2,gl2,rl2])
# u_bot=np.array([bu2,gu2,ru2])

# bl,gl,rl,bu,gu,ru=0,0,0,255,255,45

# l_object=np.array([bl,gl,rl])
# u_object=np.array([bu,gu,ru])

def callback(data):
    curr_time=time.time()
    try:
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(data,"bgr8")
        # cv_image = cv2.resize(cv_image, (800,800))
        # cv_image = np.uint8(cv_image)

        hsv1 = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv1, lb1, ub1) # green(object)

        res = cv2.bitwise_and(frame, frame, mask = mask)
        gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        _, thresh = cv2.threshold(blur, 20, 255, cv2.THRESH_BINARY)
        dilated = cv2.dilate(thresh, None, iterations=3)
        contours_object, _ = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # mask_object=cv2.inRange(cv_image,l_object,u_object)        
        # contours_object,_=cv2.findContours(mask_object, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours_object[::-1]:
            (x_object,y_object,w,h)=cv2.boundingRect(contour)
            cv2.rectangle(frame, (x_object,y_object), (x_object+w, y_object+h), (0,0,255), 2)
        
        mask = cv2.inRange(hsv1, lb2, ub2) # orange(robot)

        res = cv2.bitwise_and(frame, frame, mask = mask)
        gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        _, thresh = cv2.threshold(blur, 20, 255, cv2.THRESH_BINARY)
        dilated= cv2.dilate(thresh, None, iterations=3)
        contours_robo, _ = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # mask_bot=cv2.inRange(cv_image,l_bot,u_bot)
        # ctrs,_=cv2.findContours(mask_bot, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours_robo[::-1]:
            (x_bot,y_bot,w,h)=cv2.boundingRect(contour)
            if cv2.contourArea(contour) < 500:
                continue
            cv2.rectangle(frame, (x_bot,y_bot), (x_bot+w, y_bot+h), (0,255,0), 2)

        cv2.imshow("detection", frame)
        cv2.waitKey(1)

        global i
        global j
        global start_x_object
        global x_object_prev
        global y_object_prev
        global prev_time
        i=i+1
        j=j+1
        if j==1:
            start_x_object=x_object
        if i==3:
            time_diff=curr_time-prev_time
            x_object_diff=x_object_prev-x_object
            y_object_diff=y_object_prev-y_object
            vel_object_x=x_object_diff/time_diff
            vel_object_y=y_object_diff/time_diff
            object_h_bot=472-y_object
            D=((vel_object_y)*(vel_object_y))+1687*(object_h_bot)
            time_available=(vel_object_y+math.sqrt(D))/843
            x=start_x_object-x_object
            xx=start_x_object-x_bot
            predicted_x_object=(vel_object_x*time_available)+x
            vel_x=((predicted_x_object-xx)/(time_available))*0.012
            
            if abs(y_object-y_bot)<=25:
                vel_x=0
            
            vel_msg.linear.x= -vel_x
            
            if i!=3:
                pub1.publish(vel_msg) 
             
            print(vel_x)
            #print(vel_object_x,vel_object_y,object_h_bot)
            prev_time=curr_time
            x_object_prev=x_object
            y_object_prev=y_object
            i=0
            
    except Exception as err:
        print(err)
    
    
def listener():
    rospy.init_node('object_tracker', anonymous=True)
    global pub1 
    pub1= rospy.Publisher('cmd_vel', Twist, queue_size=10)       
    rospy.Subscriber("vehicle/camera1/image", Image, callback)
    rospy.spin()

    
if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
