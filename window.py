#!/usr/bin/env python

import rospy
import cv2
import numpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

    def __init__(self):

        cv2.startWindowThread()
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",
                                          Image, self.callback)
    '''
    def find(self, img):
	b = [([110,100,100],[130,255,255]),
		([50,100,100],[70,255,255]),
		([0,100,100],[4,255,255]),
		([30,100,100],[40,255,255])]

	for (lower,upper) in b:
            lower = numpy.array(lower, dtype = "uint8")
            upper = numpy.array(upper, dtype = "uint8")

	    mask = cv2.inRange(img, lower, upper)
	return mask
    '''
    '''
	    output = cv2.bitwise_and(cv_image,cv_image, mask= mask)
	
	    cv2.imshow("Image window", output)
            cv2.waitKey(1)
    '''

    def callback(self, data):
        cv2.namedWindow("Image window", 1)
	cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
	hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
	#self.find(hsv_img)
	'''
	output = cv2.bitwise_and(cv_image,cv_image, mask= self.find(hsv_img))
	cv2.imshow("Image window", output)
        cv2.waitKey(1)
        
	b = [([110,100,100],[130,255,255]),
		([50,100,100],[70,255,255]),
		([0,100,100],[4,255,255]),
		([30,100,100],[40,255,255])]

	for (lower,upper) in b:
            lower = numpy.array(lower, dtype = "uint8")
            upper = numpy.array(upper, dtype = "uint8")

	    mask = cv2.inRange(img, lower, upper)
	    output = cv2.bitwise_and(cv_image,cv_image, mask= mask)
	
	    cv2.imshow("Image window", output)
            cv2.waitKey(1)
	'''
	l = numpy.array([10,10,10])
	u = numpy.array([255,255,255])

        mask = cv2.inRange(hsv_img, l, u)

	res = cv2.bitwise_and(cv_image,cv_image,mask=mask)
        cv2.imshow("Image window", res)
        cv2.waitKey(1)
	
image_converter()
rospy.init_node('image_converter', anonymous=True)
rospy.spin()
cv2.destroyAllWindows()
