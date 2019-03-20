#!/usr/bin/env python

from cv2 import imread,imshow

img = imread('/home/student/Desktop/g.pug')
'''def m(Image):
    h, w, d = Image.shape
    print (h,w,d)
	
m(img)'''
imshow(img)
