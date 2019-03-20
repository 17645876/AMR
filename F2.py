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
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        self.twist = Twist()
    '''
    def m(self, Image):
	return mask
    '''

    def image_callback(self, msg):
        cv2.namedWindow("window", 1)
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	
        lower_yellow = numpy.array([10, 10, 10])
        upper_yellow = numpy.array([255, 255, 250])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
	#res = cv2.bitwise_and(image,image, mask= mask)

	h, w, d = image.shape
        search_top = 3*h/4 
        search_bot = 3*h/4 + 20 
        mask[0:search_top, 0:w] = 0 
        mask[search_bot:h, 0:w] = 0 

        M = cv2.moments(mask)
	print ('Go')
	###
        if M['m00'] > 0:
	    print('M[m00] > 0')
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
            errX = cx - w*3/4
	    errY = 3*h/4 + 10 - cy

	    self.MoveL(errX)
	    if -1<errX<1:
		self.twist.angular.z = 0
	    else:
		self.MoveA(self.twist.linear.x,errX)
            print self.twist.linear.x
	    print self.twist.angular.z
            self.cmd_vel_pub.publish(self.twist)
	else:
	    self.twist.linear.x = 0.2
	    self.twist.angular.z = 0.5
	    print('M[m00]',self.twist.linear.x)
        cv2.imshow("window", image) 
        cv2.waitKey(3) 

    def MoveL(self, Y):
	if Y >5:
	    self.twist.linear.x = 0.2
	elif Y == 5:
	    self.twist.linear.x = 0
	elif Y < 5:
       	    self.twist.linear.x = -0.2
	self.twist.linear.x = L
    
    def MoveA(self, Y, X):
	#if Y == 0:
	    #self.twist.angular.z = 0.2
	#else:
	self.twist.angular.z = -float(X) / 1000
	
    '''
    def MoveA(self, X):
	a = -float(X) / 1000
	print ('a',a)
	return a
    '''

cv2.startWindowThread()
rospy.init_node('follower')
follower = Follower()
rospy.spin()

cv2.destroyAllWindows()
