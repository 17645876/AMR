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
    	E = cv2.moments(self.mask3(hsv)) # Exploration
	    Er = cv2.moments(self.mask2(image,self.mask3(hsv)))
	    D = cv2.moments(self.mask2(image,self.mask1(hsv))) # Discovery
        
        # have target in image, but not in one meter front of robot
    	if (E['m00'] > 0 and Er['m00'] == 0):
    	    print ('___Find the target___')
    	    errX = self.ErrX1(image,E)
    	    errY = self.ErrY(image,E)
    	    self.twist.linear.x = 0.2
    	    self.MoveA1(errX)
    	
        # have target in one meter front of robot
    	elif (E['m00'] > 0 and Er['m00'] > 0):
    	    print ('___Near the target___')
    	    if self.twist.linear.x == 0:
        		print ('___Distance target less than one meter___')
                # after the target in one meter front of robot, robot turn to other direction
        		rospy.Timer(rospy.Duration(10),self.Turn())  
    	    else:
    	        errX = self.ErrX1(image,Er)
    	        errY = self.ErrY(image,Er)
    	        self.MoveL1(errY)
    	        self.MoveA1(errX)
    	    
        # no target in image, and no barries in one meter front of robot
    	elif (E['m00'] == 0 and D['m00'] == 0):
    	    print ('___No anything___')
    	    self.twist.linear.x = 0.2
    	    self.twist.angular.z = 0
            
        # no target in image, but have barries in one meter front of robot
    	elif (E['m00'] == 0 and D['m00'] > 0):
    	    print ('___Find barrier___')
    	    errX = self.ErrX2(image,D)
    	    errY = self.ErrY(image,D)
    	    self.MoveL2(errY)
    	    if errX < 0:
    	        self.MoveA3(errX)
    	    elif 0<errX<=10:
                Timer(rospy.Duration(10),self.Turn())
    	    elif errX>10:
    	        self.MoveA2(errX)

        # Publish new motion control commands to the node
    	self.cmd_vel_pub.publish(self.twist)
	    # show
	    cv2.imshow("window", res) 
        cv2.waitKey(3)
    '''
    the part of calculating error
    '''
    def ErrX1(self,img,M):
    	h, w, d = img.shape
    	ex = int(M['m10']/M['m00'])
    	ey = int(M['m01']/M['m00'])
    	cv2.circle(img, (ex, ey), 10, (255, 255, 255), -1)
    	errX = ex - w/2
    	return errX

    def ErrX2(self,img,M):
    	h, w, d = img.shape
    	ex = int(M['m10']/M['m00'])
    	ey = int(M['m01']/M['m00'])
    	cv2.circle(img, (ex, ey), 10, (255, 255, 255), -1)
    	errX = ex - w/6
    	return errX

    def ErrY(self,img,M):
    	h, w, d = img.shape
    	ex = int(M['m10']/M['m00'])
    	ey = int(M['m01']/M['m00'])
    	errY = 3*h/4 + 10 - ey
    	return errY 
	
    '''
    the part of mask
    '''
    # using to produce a binary image
    def mask1(self, img):
        lower = numpy.array([10, 10, 10])
        upper = numpy.array([255, 255, 250])
        mask = cv2.inRange(img, lower, upper)
	    return mask

    # using to detection color (blue, green, red and yellow)
    def mask2(self, img):
    	b = [([110,100,100],[130,255,255]),([50,100,100],[70,255,255]),([0,100,100],[4,255,255]),([30,100,100],[40,255,255])]
    	for (lower,upper) in b:
                lower = numpy.array(lower, dtype = "uint8")
                upper = numpy.array(upper, dtype = "uint8")
    	    mask = cv2.inRange(img, lower, upper)
    	return mask
    
    # using to check the item which in one meter front of robot
    def mask3(self, img, mask):
	    h, w, d = img.shape
        search_top = 3*h/4 
        search_bot = 3*h/4 + 20 
        mask[0:search_top, 0:w] = 0 
        mask[search_bot:h, 0:w] = 0
	    return mask
    
    '''
    the part for moving decide
    '''
    # contral the linear when robot moving to target, and keep distance
    def MoveL1(self, Y):
    	if Y >2:
    	    self.twist.linear.x = 0.2
    	elif Y == 2:
    	    self.twist.linear.x = 0
    	elif Y < 2:
            f.twist.linear.x = -0.2
    # contral the linear when robots' moving when robot find barries
    def MoveL2(self, Y):
    	if Y >=5:
    	    self.twist.linear.x = 0.2
    	elif 2 <= Y < 5:
            f.twist.linear.x = 0.1
    	elif Y < 2:
            f.twist.linear.x = -0.05
            
    # contral the angular when
    def MoveA1(self, X):
        f.twist.angular.z = -float(X) / 1000
    def MoveA2(self, X):
    	if self.twist.linear.x < 0.2:
    	    self.twist.angular.z = 0.2
    	else:
            self.twist.angular.z = float(X) / 1000
    def MoveA3(self, X):
    	if self.twist.linear.x < 0.2:
    	    self.twist.angular.z = -0.2
    	else:
            self.twist.angular.z = -float(X) / 1000
            
    '''
    the part for restart discover
    '''
    def Turn(self):
    	rospy.Timer(rospy.Duration(5),self.Break())
    	self.twist.linear.x = 0
    	self.twist.angular.z = 0.2
    	self.cmd_vel_pub.publish(self.twist)
    def Break(self):
    	self.twist.linear.x = 0
    	self.twist.angular.z = 0
    	self.cmd_vel_pub.publish(self.twist)
	 

cv2.startWindowThread()
rospy.init_node('follower')
follower = Follower()
rospy.spin()

cv2.destroyAllWindows()
