#!/home/companion/tensorflow_ros/bin/python
# coding: utf-8

import os
import sys
import random
import math
import cv2
import time
import numpy as np
import itertools
import colorsys
#import IPython.display
import pandas as pd

from math import sin, cos
from skimage.measure import find_contours
from PIL import Image

def berry_vs():
	# run the arm camera
	os.system('fswebcam -d /dev/video0 -q -r 640x480 --no-banner --jpeg 95 -F 9 vs.jpg')
	img = cv2.imread('vs.jpg')
	mask, center = blob_search(img, 'berry')
	cv2.waitKey(1)
	# x y in pixel,z calculated in cm
	xd = 320
	yd = 240
	delta=[]
	n = np.count_nonzero(mask)
	if n != 0:
		# calculate the z distance between berry and the rigid_arm_position_desired based on pixels
		z = (1/(292300.)*n)**(-1/1.918)
		# calculate the center of the berry
		M = cv2.moments(mask)
		x = int(M["m10"] / M["m00"])
		y = int(M["m01"] / M["m00"])
		# z is calculated using expoential fit
		delta = []
		delta.append(x-xd)
		delta.append(y-yd)
		delta.append(z)
	print(delta)
	return delta



def cal_error(t1, t2, t3, d1, d2, d3):
	# transformation matrix with end effector error
	x = d1*cos(t2 - t3)*cos(t1) - 0.267*cos(t1)*sin(t2) - 1.0*d2*sin(t1) - 0.02775*cos(t1)*sin(t2)*sin(t3) - 0.0213*sin(t1) - 1.0*d3*sin(t2 - t3)*cos(t1) - 0.02775*cos(t1)*cos(t2)*cos(t3) + 0.4407*cos(t1)*cos(t2)*sin(t3) - 0.4407*cos(t1)*cos(t3)*sin(t2)
	y = 0.0213*cos(t1) - 0.267*sin(t1)*sin(t2) + d2*cos(t1) + 0.4407*cos(t2)*sin(t1)*sin(t3) - 0.4407*cos(t3)*sin(t1)*sin(t2) - 0.02775*sin(t1)*sin(t2)*sin(t3) + d1*cos(t2 - t3)*sin(t1) - 1.0*d3*sin(t2 - t3)*sin(t1) - 0.02775*cos(t2)*cos(t3)*sin(t1)
	z = 0.44157281675845944219620991630904*cos(t2 - t3 + 0.062884980792608442735334216260151) + 0.267*cos(t2) + d3*cos(t2 - t3) + d1*sin(t2 - t3) + 0.203
	print(x,y,z)
	return [x, y, z]


def blob_search(image_raw, color):

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # Filter by Color
    params.filterByColor = False

    # Filter by Area.
    params.filterByArea = False
    params.minArea = 50

    # Filter by Circularity
    params.filterByCircularity = False
    #params.minCircularity = 0.5

    # Filter by Inerita
    params.filterByInertia = False

    # Filter by Convexity
    params.filterByConvexity = False
    params.minConvexity = 0.5

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)

    if color == "berry":
        lower = (0, 50, 50)
        upper = (10, 255, 255)

    # Define a mask using the lower and upper bounds of the target color
    mask_image = cv2.inRange(hsv_image, lower, upper)
    keypoints = detector.detect(mask_image)
    # Find blob centers in the image coordinates
    blob_image_center = []
    num_blobs = len(keypoints)
    for i in range(num_blobs):
        blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))

    blob_image_center = fuse(blob_image_center, 250)
    output_masks=[]
    for i in range(len(blob_image_center)):
        z = np.zeros((480, 848))
        z[int(blob_image_center[i][1])][int(blob_image_center[i][0])] = 1
        output_masks.append(z)

    # Draw the keypoints on the detected block
    im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints, outImage=np.array([]), color=(10, 255, 255))

    x_y = []

    if(num_blobs == 0):
        #print("No block found!")
        f = 0 # do nothing
    else:
        # Convert image coordinates to global world coordinate using IM2W() function
        for i in range(len(blob_image_center)):
            x_y.append((blob_image_center[i][0], blob_image_center[i][1]))

    cv2.namedWindow("Camera View")
    cv2.imshow("Camera View", image_raw)
    cv2.namedWindow("Mask View")
    cv2.imshow("Mask View", mask_image)
    cv2.namedWindow("Keypoint View")
    cv2.imshow("Keypoint View", im_with_keypoints)

    return np.asanyarray(mask_image), blob_image_center

def dist2(p1, p2):
    return (p1[0]-p2[0])**2 + (p1[1]-p2[1])**2

def fuse(points, d):
    ret = []
    d2 = d * d
    n = len(points)
    taken = [False] * n
    for i in range(n):
        if not taken[i]:
            count = 1
            point = [points[i][0], points[i][1]]
            taken[i] = True
            for j in range(i+1, n):
                if dist2(points[i], points[j]) < d2:
                    point[0] += points[j][0]
                    point[1] += points[j][1]
                    count+=1
                    taken[j] = True
            point[0] /= count
            point[1] /= count
            ret.append((point[0], point[1]))
    return ret

cal_error(np.deg2rad(-10), np.deg2rad(-30), np.deg2rad(90), 0, 0, 0)
