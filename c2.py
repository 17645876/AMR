#!/usr/bin/env python

import rospy
import cv2
import numpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String


class image_converter:

    def __init__(self):

        cv2.startWindowThread()
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",
                                          Image, self.callback)
        # self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",
        #                                   Image, self.callback)

    def callback(self, data):
        cv2.namedWindow("Image window", 1)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e

        bgr_thresh = cv2.inRange(cv_image,
                                 numpy.array((200, 230, 230)),
                                 numpy.array((255, 255, 255)))

        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        hsv_thresh = cv2.inRange(hsv_img, numpy.array([110,50,50]), numpy.array([130,255,255]))

    	# Threshold the HSV image to get only blue colors
   	mask = cv2.inRange(hsv_img, numpy.array([110,50,50]), numpy.array([130,255,255]))

   	# Bitwise-AND mask and original image
   	res = cv2.bitwise_and(cv_image,cv_image, mask= mask)

        print numpy.mean(hsv_img[:, :, 0])
        print numpy.mean(hsv_img[:, :, 1])
        print numpy.mean(hsv_img[:, :, 2])

        _, bgr_contours, hierachy = cv2.findContours(
            bgr_thresh.copy(),
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE)

        _, hsv_contours, hierachy = cv2.findContours(
            hsv_thresh.copy(),
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE)
        for c in hsv_contours:
            a = cv2.contourArea(c)
            if a > 100.0:
                cv2.drawContours(cv_image, c, -1, (255, 0, 0))
        print '===='
	cv2.imshow("Image window", cv_image)
	cv2.imshow("res", res)
	self.p=rospy.Publisher('/msgs', String)
	s = str(numpy.mean(hsv_img[:, :, 0]) + numpy.mean(hsv_img[:, :, 1]) + numpy.mean(hsv_img[:, :, 2]))
	self.p.publish(s)
        cv2.waitKey(1)


image_converter()
rospy.init_node('image_converter', anonymous=True)
rospy.spin()
cv2.destroyAllWindows()
