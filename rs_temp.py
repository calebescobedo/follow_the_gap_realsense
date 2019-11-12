import pyrealsense2 as rs
import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt 
import rs_util as util

pipeline = util.setup_realsense(rs)


# Set Variables used in camera loop
depth_lower_bound = 275 
depth_image_size = 75
depth_thresh = 15
depth_upper_bound = depth_lower_bound - depth_image_size


# The video loop
try:
	while True:

		# Wait for a coherent pair of frames: depth and color
		depth_frame, color_frame = util.get_frames(pipeline)
		if not depth_frame or not color_frame:
			continue

		depth_image = np.asanyarray(depth_frame.get_data())
		color_image = np.asanyarray(color_frame.get_data())
		depth_colormap = cv.applyColorMap(cv.convertScaleAbs(depth_image, alpha=0.03), cv.COLORMAP_JET)
		depth_colormap_cut = depth_colormap[depth_upper_bound:depth_lower_bound,:]
		avg_depth = np.transpose(np.nanmean(depth_colormap_cut, axis=0))

		prev_pixel = avg_depth[0,0]
		start = 0
		end = 0
		obstacles = []

		# Get list of all objects in the image
		for idx, cur_pixel in enumerate(avg_depth[0,:]):
			if idx == len(avg_depth[0,:]) - 1:	
				obstacles.append([start, end])
			elif abs(prev_pixel - cur_pixel) < depth_thresh:
				end += 1
			else:
				obstacles.append([start, end])
				start = idx
				end = idx
			prev_pixel = cur_pixel
		

		if len(obstacles) > 4:
			avg_vals = [np.nanmean(avg_depth[0,row]) for row in obstacles]
			avg_np = np.array(avg_vals)
			obs_np = np.array(obstacles)
			minimum_index = avg_np.argsort()[0:3]	
			obstacles = list(obs_np[minimum_index])	
		

		for row in obstacles:
			depth_colormap = cv.rectangle(depth_colormap, (row[0], 200), (row[1], 275), (0, 0 ,255), 2)		


		# Show images
		cv.imshow('RealSense', depth_colormap)
		cv.imshow('cut_depth', depth_colormap_cut)
		cv.waitKey(1)

finally:
    # Stop streaming
    pipeline.stop()
