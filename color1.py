#!/usr/bin/env python

import numpy
import cv2

i = cv2.imread('/Desktop/color.gif')
b = [([110,100,100],[130,255,255]),([50,100,100],[70,255,255]),([0,100,100],[10,255,255]),([80,100,100],[100,255,255])]

cv2.namedWindow("Image window", 1)
#image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
hsv = cv2.cvtColor(i, cv2.COLOR_BGR2HSV)

for (lower,upper) in b:
    lower = numpy.array(lower, dtype = "uint8")
    upper = numpy.array(upper, dtype = "uint8")
    mask = cv2.inRange(hsv, lower, upper)
    output = cv2.bitwise_and(image,image, mask= mask)
	
    cv2.imshow("Image window", output)
    cv2.waitKey(1)
	
