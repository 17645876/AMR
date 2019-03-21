#!/usr/bin/env python

import numpy
import cv2

green = numpy.uint8([[[102,102,102 ]]])
hsv_green = cv2.cvtColor(green,cv2.COLOR_BGR2HSV)
print hsv_green
