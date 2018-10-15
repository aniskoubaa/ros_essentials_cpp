#!/usr/bin/env python

#import numpy: the data structure that will handle an image
import numpy as np

#import openCV
import cv2
 

image_name = "flower"

print 'read an image from file'
img = cv2.imread("images/"+image_name+".jpg")

print 'create a window holder for the image'
cv2.namedWindow("Image",cv2.WINDOW_NORMAL)

print 'display the image'
cv2.imshow("Image",img)

print 'press a key inside the image to make a copy'
cv2.waitKey(0)

print 'image copied to folder images/copy/'
cv2.imwrite("images/copy/"+image_name+"-copy.jpg",img)
