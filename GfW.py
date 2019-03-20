#!/usr/bin/env python

import numpy

import cv2
import cv_bridge
import rospy

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image,
                                          self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist,
                                           queue_size=1)
        self.twist = Twist()

    def image_callback(self, msg):
        cv2.namedWindow("window", 1)
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') # ROS to OpenCV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) # RGB & HSV
        lower_yellow = numpy.array([10, 10, 10]) # binary
        upper_yellow = numpy.array([255, 255, 250]) # binary
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow) # binary
        h, w, d = image.shape # search
        search_top = 3*h/4 # search
        search_bot = 3*h/4 + 20 # search
        mask[0:search_top, 0:w] = 0 # search
        mask[search_bot:h, 0:w] = 0 # search
	mask[search_top:search_bot, 0:w/2] = 0 # search
        M = cv2.moments(mask) # blob
        if M['m00'] > 0: # blob
            cx = int(M['m10']/M['m00']) # blob
            cy = int(M['m01']/M['m00']) # blob
            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1) # red circle
	    ###
            errX = cx - w/2 
	    errY = 3*h/4 + 10 - cy
	    if errY >5:
	    	self.twist.linear.x = 0.2
	    elif errY == 5:
	   	self.twist.linear.x = 0
	    elif errY < 5:
       	   	self.twist.linear.x = -0.2
            self.twist.linear.x = 0.2 
	    self.twist.angular.z = -float(errX) / 1000
            print self.twist.angular.z
            self.cmd_vel_pub.publish(self.twist) 
        cv2.imshow("window", image) 
        cv2.waitKey(3) 


cv2.startWindowThread()
rospy.init_node('follower')
follower = Follower()
rospy.spin()

cv2.destroyAllWindows()
