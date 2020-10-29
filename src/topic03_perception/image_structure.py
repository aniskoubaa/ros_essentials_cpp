#!/usr/bin/env python

#import numpy: the data structure that will handle an image
import numpy as np

#import openCV
import cv2

image_name = "blackwhite"

print ('read an image from file')
img = cv2.imread("images/"+image_name+".jpg")

print ('display the content of the image')
print (img)
print ('In Python, an image is stored in a numpy array. Numpy is library used for scientific computing of multi-dimensional arrays and matrices.')

print ('we can determine several features of the images using numpy array properties')
print ('type of an image type(img): %s'%type(img))
print ('size of the image img.size: %d'%img.size)
print ('length of the image (number of pixel in the vertical direction) len(img): %d'%len(img))
print ('shape of an image (length in pixe, width in pixel, number of color) img.shape (%d,%d,%d)'%img.shape)
print ('image length (also height) img.shape[0]: %d'%img.shape[0])
print ('image width img.shape[1]: %d'%img.shape[1])

height, width, channels = img.shape
print ('height = %d'%height)
print ('width = %d'%width)
print ('channels = %d'%channels)

print ('number of colors per pixel img.shape[2]: %d'%img.shape[2])
print ('number of pixels: %d'%(img.shape[0]*img.shape[1]))
print ('type of the image img.dtype: %s'%img.dtype)
print ('sub-image at row [10] (img[10])')
print (img[10])
print ('shape of sub-image at row [0] (img[10].shape)')
print (img[10].shape)
print ('pixel at raw 10 and column 5 (img[10, 5])')
print (img[10, 5])
print (img[10] [5])
print ('pixel at raw 0 and column 0 (img[0, 0])')
print (img[0, 0])
print (img[0] [0])

print ('you can see a single channel in the image, for example only the first channel')
print (img[:, :, 0])

