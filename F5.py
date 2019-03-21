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

    def image_callback(self, msg):
        cv2.namedWindow("window", 1)
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	h, w, d = image.shape
	#self.mask2(image,self.mask1(hsv))
	#res = cv2.bitwise_and(image,image,mask=self.mask2(image,self.mask1(hsv)))
	#cv2.imshow("Image window", res)
        #cv2.waitKey(1)
	E = cv2.moments(self.mask3(hsv)) # Exploration
	Er = cv2.moments(self.mask2(image,self.mask3(hsv)))
	D = cv2.moments(self.mask2(image,self.mask1(hsv))) # Discovery
	
	if (E['m00'] > 0):
	    ex = int(E['m10']/E['m00'])
	    ey = int(E['m01']/E['m00'])
	    cv2.circle(image, (ex, ey), 10, (0, 0, 255), -1)
	    errX = ex - w/2
	    self.twist.angular.z = -float(errX) / 1000
	    print self.twist.angular.z
	    self.cmd_vel_pub.publish(self.twist)
	    
	    #ex = int(Er['m10']/Er['m00'])
            #ey = int(Er['m01']/Er['m00'])
	'''
	if (D['m00'] ==0):
	    self.twist.linear.x = 0.02
	    self.twist.angular.z = 0.05
	    self.cmd_vel_pub.publish(self.twist)
	else:
            cx = int(D['m10']/D['m00'])
            cy = int(D['m01']/D['m00'])
            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
            errX = cx - w/2
	    errY = 3*h/4 + 10 - cy
            print('M[m00] > 0')
	    self.MoveL(errX)
	    if -1<errX<1:
	    	self.twist.angular.z = 0
	    else:
	    	self.MoveA(self.twist.linear.x,errX)
            print self.twist.linear.x
	    print self.twist.angular.z
	    self.cmd_vel_pub.publish(self.twist)
	'''
	cv2.imshow("window", image) 
        cv2.waitKey(3) 
	
    ''''''
    def mask1(self, img):
        lower = numpy.array([10, 10, 10])
        upper = numpy.array([255, 255, 250])
        mask = cv2.inRange(img, lower, upper)
	return mask

    def mask2(self, img, mask):
	h, w, d = img.shape
        search_top = 3*h/4 
        search_bot = 3*h/4 + 20 
        mask[0:search_top, 0:w] = 0 
        mask[search_bot:h, 0:w] = 0
	return mask

    def mask3(self, img):
	b = [([110,100,100],[130,255,255]),
		([50,100,100],[70,255,255]),
		([0,100,100],[4,255,255]),
		([30,100,100],[40,255,255])]
	for (lower,upper) in b:
            lower = numpy.array(lower, dtype = "uint8")
            upper = numpy.array(upper, dtype = "uint8")

	    mask = cv2.inRange(img, lower, upper)
	return mask

    def MoveL(self, Y):
	if Y >5:
	    self.twist.linear.x = 0.2
	elif Y == 5:
	    self.twist.linear.x = 0
	elif Y < 5:
       	    self.twist.linear.x = -0.2
	self.twist.linear.x
    
    def MoveA(self, Y, X):
	self.twist.angular.z = -float(X) / 1000

cv2.startWindowThread()
rospy.init_node('follower')
follower = Follower()
rospy.spin()

cv2.destroyAllWindows()
