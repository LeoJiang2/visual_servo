#!/usr/bin/env python
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
import IPython.display
import pyrealsense2 as rs 
import pandas as pd

from skimage.measure import find_contours
from matplotlib.patches import Polygon
from PIL import Image

# Root directory of the project
ROOT_DIR = os.path.abspath("./")


# get_ipython().run_line_magic('matplotlib', 'inline')


# Local path to trained weights file
MODEL_PATH = os.path.join(ROOT_DIR, "mask_rcnn_cherry_tomato.h5")

# Directory of images to run detection on
IMAGE_DIR = os.path.join(ROOT_DIR, "images")

sys.path.append(ROOT_DIR)  # To find local version of the library

# Import Mask RCNN
import mrcnn.model as modellib
from mrcnn import visualize
from mrcnn.model import log
from mrcnn import utils
from mrcnn.config import Config



# berry Class names
# Index of the class in the list is its ID. For example, to get ID of
class_names = ['Background','cherry tomato']
ftitle = time.strftime("%Y%m%d", time.localtime())
VISUALIZE = True
SAVE_RESULTS = True
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
        if y >= quartileSet[0] and y <= quartileSet[1]:
            resultList.append(y)
    return resultList

def XYZ_calculation(depth_image,masks,Number):
    for c in range(Number):
        depth_array = np.where(masks[:, :, c] == 1,depth_image,masks[:, :, c])
        depth_value =  depth_array[np.nonzero(depth_array)]
        depth_removeoutliers = removeOutliers(depth_value,1)
        depth_mean = np.mean(depth_removeoutliers)*depth_scale*1000  # convert to mm

        M = cv2.moments(depth_array)
        cX= int(M["m10"] / M["m00"])
        cY= int(M["m01"] / M["m00"])    
        XYZ_point = rs.rs2_deproject_pixel_to_point(depth_intrin, [cX,cY], depth_mean)
        XYZ_coordinates.append(XYZ_point)
    return XYZ_coordinates

class InferenceConfig(Config):
    # Set batch size to 1 since we'll be running inference on
    # one image at a time. 
    NAME = "cherry tomato"

    # Number of classes (including background)
    NUM_CLASSES = 1 + 1  # Background + cherry tomato
    GPU_COUNT = 1
    IMAGES_PER_GPU = 1

# Creat config
config = InferenceConfig()

# Create model object in inference mode.
model = modellib.MaskRCNN(mode="inference", model_dir=MODEL_PATH, config=config)

# Load weights trained on cherry tomatoes
model.load_weights(MODEL_PATH, by_name=True)


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

try:
    while True:
        
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)
        
        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() 

        color_frame = aligned_frames.get_color_frame()
        
        depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
#         color_intrin = color_frame.profile.as_video_stream_profile().intrinsics

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
        
        # Manual data collection and testing (Press space,ascii 32)
        # Tap the Space to save the image 
        
        if c == 32:
            # start time
            start = time.time()
            # Count the number of csv files in the folder, 
            # so that the recollection will not overwrite the existing pictures after closing the software 
            count = 0
            for filename in os.listdir('./results/'):
                if filename.endswith('.csv'):
                    count +=1
                        
            # Convert images to numpy arrays
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
    
            results = model.detect([color_image], verbose=1)

            # Display results
            ax = get_ax(1)
            r = results[0]

            # Number of instances
            N = r['rois'].shape[0]
            XYZ_coordinates = []
            XYZ_coordinates = XYZ_calculation(depth_image,r['masks'],N)
            XYZ_coordinates = pd.DataFrame(XYZ_coordinates,columns=list('xyz'))
            print(XYZ_coordinates)
            print(time.time()-start)
            if VISUALIZE == True:
                visualize.display_instances(color_image, r['rois'], r['masks'], r['class_ids'], 
                                            'cherry tomato',(XYZ_coordinates['x']),
                                            (XYZ_coordinates['y']),(XYZ_coordinates['z']), ax=ax,  
                                            title="XYZ Coordinates",
                                            figurepath=('./results/'+ftitle+'-XYZ-coordinates-{}.png'.format(count + 1)))
            if SAVE_RESULTS == True:    
                # Save the color image
                cv2.imwrite('./results/'+ftitle+'-raw-figure-{}.png'.format(count + 1), color_image)
                
                XYZ_coordinates.to_csv('./results/'+ftitle+'-XYZ-coordinates-{}.csv'.format(count + 1))
            
                    
     	# Press ESC (ascii 27）to stop collection and stop the loop
        if c == 27:
            cv2.destroyAllWindows()
            break 

finally:   

    # Stop streaming
    pipeline.stop()



