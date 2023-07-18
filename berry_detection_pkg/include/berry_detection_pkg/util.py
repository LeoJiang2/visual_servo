#!/home/companion/tensorflow_ros/bin/python
# coding: utf-8

import argparse
import os
import sys
import random
import math
import cv2
import time
import numpy as np
import skimage.io
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.lines as lines
import itertools
import colorsys
#import IPython.display
import pyrealsense2 as rs
import pandas as pd

from skimage.measure import find_contours
from matplotlib.patches import Polygon
from PIL import Image


# Import Mask RCNN
# import mrcnn.model as modellib
# from mrcnn import visualize
# from mrcnn.model import log
# from mrcnn import utils
# from mrcnn.config import Config


def getImageNdarrayVar(image):
    img2gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    imageVar = cv2.Laplacian(img2gray, cv2.CV_64F).var()
    return imageVar

def get_ax(rows=1, cols=1, size=16):
    """Return a Matplotlib Axes array to be used in
    all visualizations in the notebook. Provide a
    central point to control graph sizes.

    Adjust the size attribute to control how big to render images
    """
    _, ax = plt.subplots(rows, cols, figsize=(size*cols, size*rows))
    return ax

def removeOutliers(x, outlierConstant):
    a = np.array(x)
    upper_quartile = np.percentile(a, 75)
    lower_quartile = np.percentile(a, 25)
    IQR = (upper_quartile - lower_quartile) * outlierConstant
    quartileSet = (lower_quartile - IQR, upper_quartile + IQR)
    resultList = []
    for y in a.tolist():
       # if y >= quartileSet[0] and y <= quartileSet[1]:
        if y >= quartileSet[0]:
            resultList.append(y)
    return resultList

#  for ml model
# def XYZ_calculation(XYZ_coordinates, depth_intrin, depth_scale, depth_image,masks,Number):
#
#     for c in range(Number):
#         depth_array = np.where(masks[:, :, c] == 1,depth_image,masks[:, :, c])
#         depth_value =  depth_array[np.nonzero(depth_array)]
#         depth_removeoutliers = removeOutliers(depth_value,1)
#         depth_mean = np.mean(depth_removeoutliers)*depth_scale*1000  # convert to mm
#
#         M = cv2.moments(depth_array)
#         cX= int(M["m10"] / M["m00"])
#         cY= int(M["m01"] / M["m00"])
#         XYZ_point = rs.rs2_deproject_pixel_to_point(depth_intrin, [cX,cY], depth_mean)
#         XYZ_coordinates.append(XYZ_point)
#     return XYZ_coordinates

# for blob detection
# def XYZ_calculation(XYZ_coordinates, depth_intrin, depth_scale, depth_image,masks, Number, center):
#     for c in range(Number):
#         depth_array = np.where(masks[c, :, :] == 1,depth_image,masks[c, :, :])
#         depth_value =  depth_array[np.nonzero(depth_array)]
#         # depth_removeoutliers = removeOutliers(depth_value,1)
#         depth_mean = np.mean(depth_value)*depth_scale*1000  # convert to mm
#         XYZ_point = rs.rs2_deproject_pixel_to_point(depth_intrin, [int(center[c][0]), int(center[c][1])] , depth_mean)
#         XYZ_coordinates.append(XYZ_point)
#     return XYZ_coordinates

def XYZ_calculation(XYZ_coordinates, depth_intrin, depth_scale, depth_image,masks, center):
    if len(center) == 0:
        return XYZ_coordinates
    depth_array = np.where(masks[ :, :] == 1,depth_image,masks[ :, :])
    depth_value =  depth_array[np.nonzero(depth_array)]
    depth_mean = np.mean(depth_value)*depth_scale*1000  # convert to mm
    XYZ_point = rs.rs2_deproject_pixel_to_point(depth_intrin, [int(center[0][0]), -(480-int(center[0][1]))] , depth_mean)
    XYZ_coordinates.append(XYZ_point)
    return XYZ_coordinates

# class InferenceConfig(Config):
#     # Set batch size to 1 since we'll be running inference on
#     # one image at a time.
#     NAME = "cherry tomato"
#
#     # Number of classes (including background)
#     NUM_CLASSES = 1 + 1  # Background + cherry tomato
#     GPU_COUNT = 1
#     IMAGES_PER_GPU = 1



def find_berry_points():
    #Root directory of the project
    ROOT_DIR = os.path.abspath("./")
    cwd = os.getcwd()
    model_path = cwd[:-4]
    print('util path')
    print(model_path)
    model_path1 = model_path + 'ts_ws/'

    # Local path to trained weights file
    MODEL_PATH = os.path.join(model_path1, "mask_rcnn_cherry_tomato.h5")


    # Local path to trained weights file

    # Directory of images to run detection on
    IMAGE_DIR = os.path.join(ROOT_DIR, "images")

    sys.path.append(ROOT_DIR)  # To find local version of the library
    # berry Class names
    # Index of the class in the list is its ID. For example, to get ID of
    class_names = ['Background','cherry tomato']
    ftitle = time.strftime("%Y%m%d", time.localtime())
    VISUALIZE = True
    SAVE_RESULTS = True

    # Creat config
    # config = InferenceConfig()
    #
    # # Create model object in inference mode.
    # model = modellib.MaskRCNN(mode="inference", model_dir=MODEL_PATH, config=config)
    #
    # # Load weights trained on cherry tomatoes
    # model.load_weights(MODEL_PATH, by_name=True)


    # Configure depth and color streams
    pipeline = rs.pipeline()

    # Create a config and configure the pipeline to stream
    # different resolutions of color and depth streams
    config = rs.config()
    config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)                 # depth data
    config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)  #1280, 720    # RGB data

    # Start streaming
    profile = pipeline.start(config)

    # Getting the depth sensor's depth scale
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    #  Remove the background of objects more than clipping_distance_in_meters meters away
    clipping_distance_in_meters = 10 # 10 meter
    clipping_distance = clipping_distance_in_meters / depth_scale

    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align_to = rs.stream.color
    align = rs.align(align_to)

    # Streaming loop


    loop_count = 4
    XYZ_coordinates = []
    while loop_count > 1:

        # Get frameset of color and depth
        for i in range(10):
            frames = pipeline.wait_for_frames()

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame()

        color_frame = aligned_frames.get_color_frame()

        depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
        # color_intrin = color_frame.profile.as_video_stream_profile().intrinsics

	# Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Remove background - Set pixels further than clipping_distance to grey
        grey_color = 153
        depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
        bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)


        # Render images
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        images = np.hstack((bg_removed, depth_colormap))

        # Resize the image size for better visualization in the window
        images  = cv2.resize(images, (1280, 460))

        if VISUALIZE == True:
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', images)
        c = cv2.waitKey(1)
        mask, num, center = blob_search(color_image, 'berry')
        # Manual data collection and testing (Press space,ascii 32)
        # Tap the Space to save the image

	# start time
        start = time.time()
        # Count the number of csv files in the folder,
        # so that the recollection will not overwrite the existing pictures after closing the software
        # Convert images to numpy arrays
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # results = model.detect([color_image], verbose=1)
        #
        # r = results[0]
        # print_rois = r['rois']
        # print(r)
        #
        # # Number of instances
        # N = r['rois'].shape[0]
        # XYZ_coordinates = []
        # XYZ_coordinates = XYZ_calculation(XYZ_coordinates, depth_intrin, depth_scale, depth_image,r['masks'],N)
        #
        # XYZ_coordinates = XYZ_calculation(XYZ_coordinates, depth_intrin, depth_scale, depth_image, mask, num, center)
        XYZ_coordinates = []
        XYZ_coordinates = XYZ_calculation(XYZ_coordinates, depth_intrin, depth_scale, depth_image, mask, center)
        filtered_XYZ = []
        for ele in range(len(XYZ_coordinates)):
            if XYZ_coordinates[ele][2] < 700:
                filtered_XYZ.append(XYZ_coordinates[ele])

        # if (len(filtered_XYZ) != 0):
        #     x = XYZ_coordinates[0][0]/1000.0
        #     y = XYZ_coordinates[0][1]/1000.0
        #     z = XYZ_coordinates[0][2]/1000.0
        #     k = kin.Kinematics()
        #     point_base = k._camera2arm_base(point_camera=[x,y,z], pan_d=0, tilt_d=0)
        #     print(point_base)
        # print(f'Util: Berry Detection Results = {filtered_XYZ}')
        # print(time.time()-start)
        cv2.imwrite(model_path1 + 'result.png', color_image)
        cv2.destroyAllWindows()
        loop_count -= 1
        # loop_count = 0
    return(filtered_XYZ)

def blob_search(image_raw, color):

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # Filter by Color
    params.filterByColor = False

    # Filter by Area.
    params.filterByArea = False
    params.minArea = 100

    # Filter by Circularity
    params.filterByCircularity = False
    #params.minCircularity = 0.5

    # Filter by Inerita
    params.filterByInertia = False

    # Filter by Convexity
    params.filterByConvexity = False

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)

    if color == "white":
        lower = (0,0,175)
        upper = (5,5,255)

    elif color == "purple":
        lower = (125,50,50)
        upper = (145, 255, 240)

    elif color == "berry":
        lower = (0, 150, 80)
        upper = (5, 255, 255)

    elif color == "green":
        lower = (120, 50, 50)
        upper = (180, 255, 255)

    elif color == "yellow":
        lower = (22, 50, 50)
        upper = (45, 255, 240)

    # Define a mask using the lower and upper bounds of the target color
    mask_image = cv2.inRange(hsv_image, lower, upper)
    keypoints = detector.detect(mask_image)

    # Find blob centers in the image coordinates
    blob_image_center = []
    num_blobs = len(keypoints)
    for i in range(num_blobs):
        blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))

    blob_image_center = fuse(blob_image_center, 30)
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

    return np.asanyarray(mask_image)/255, len(blob_image_center), blob_image_center

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
