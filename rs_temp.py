import sys
# Need this in order to use realsense on my mac
sys.path.append('/usr/local/lib') 
import pyrealsense2 as rs
import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt 
import matplotlib.animation as animation
import rs_util as util


# Setup figure and realsense
fig, ax = util.setup_figure(plt)
pipeline = util.setup_realsense(rs)

# Set Variables used in camera loop
depth_lower_bound = 275 
depth_image_height = 75
depth_thresh = 250
depth_upper_bound = depth_lower_bound - depth_image_height


# The video loop
try:
	while True:

		# Wait for a coherent pair of frames: depth and color
		depth_frame, color_frame = util.get_frames(pipeline)
		if not depth_frame or not color_frame:
			continue

		# Convert realsense frames to numpy array
		depth_image = np.asanyarray(depth_frame.get_data())
		color_image = np.asanyarray(color_frame.get_data())
		
		# Alter the images for viewing and processing
		depth_image_cut = depth_image[depth_upper_bound:depth_lower_bound,:]
		depth_colormap = cv.applyColorMap(cv.convertScaleAbs(depth_image, alpha=0.04), cv.COLORMAP_JET)
		median_depth = np.transpose(np.nanmedian(depth_image_cut, axis=0))


		# From the median_depth get the objects that exist in the scene
		obstacles = util.get_obstacles(median_depth, depth_thresh)
		
		circles = util.get_obstacle_circles(depth_colormap, median_depth, obstacles)

		util.plot_circles(circles, plt, ax, fig)

		# Show images
		cv.imshow('RealSense', depth_colormap)
		cv.waitKey(1)

finally:
    pipeline.stop()
