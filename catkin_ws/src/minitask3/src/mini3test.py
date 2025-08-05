#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2, cv_bridge
import numpy as np
import time
class Follower:
    lower=[0,50,0]
    upper=[20,260,20]
    lower=np.array(lower,dtype="uint8")
    upper=np.array(upper,dtype="uint8")
    
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("original", 1)
        #cv2.namedWindow("green",2)
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.twist = Twist()
    def image_callback(self, msg):
       #rospy.loginfo(msg)
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        (h, w) = image.shape[:2]
        #image_resized=image
        image_resized = cv2.resize(image, (w/4,h/4))
        #mask = cv2.inRange(image, self.lower, self.upper)
        #output = cv2.bitwise_and(image, image, mask = mask)
        #image2= np.hstack([image, output])
        #image2_resized=cv2.resize(image2,(w/4,h/4))
        #slip the color
        hsv = cv2.cvtColor(image_resized, cv2.COLOR_BGR2HSV)
        lower_green = np.array([35,43,46])
        upper_green = np.array([77, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)
        #masked = cv2.bitwise_and(image_resized, image_resized, mask=mask)
        
        h, w, d = image_resized.shape       
        print("h=",h,"w=",w)
        search_top = 1*h/4
        print("search_top", search_top)
        search_bot = 3*h/4 + 20
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        print(255-mask[100:110,100:110])
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(image_resized, (cx, cy), 20, (0,0,255), -1)
            err = cx - w/2
            self.twist.linear.x = 0.2
            self.twist.angular.z = -float(err) / 100
            self.cmd_vel_pub.publish(self.twist)
        
        cv2.imshow("original",image_resized)
        cv2.waitKey(3)



'''
        color=[([0,30,0],[35,150,35])]
        for (lower,upper) in color:
            lower = np.array(lower, dtype = "uint8")
            upper = np.array(upper, dtype = "uint8")
            mask = cv2.inRange(image_resized, lower, upper)
            output = cv2.bitwise_and(image_resized, image_resized, mask = mask)
            cv2.imshow("green",  output)
            cv2.waitKey(0)
'''


rospy.init_node('follower')
follower = Follower()
rospy.spin()
