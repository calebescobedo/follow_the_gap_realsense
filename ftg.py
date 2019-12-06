import sys
# Need this in order to use realsense on my mac
sys.path.append('/usr/local/lib') 
import pyrealsense2 as rs
import numpy as np
import cv2 as cv
import ftg_util as util
import math


pipeline = util.setup_realsense(rs)

# Set variables used in camera loop
depth_lower_bound = 275
depth_image_height = 75
depth_thresh = 100
depth_upper_bound = depth_lower_bound - depth_image_height

# Init the previous two frames
depth_2 = np.array([])
depth_1 = np.array([])

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
		avg_depth = depth_image_cut
		depth_colormap = cv.applyColorMap(cv.convertScaleAbs(depth_image, alpha=0.04), cv.COLORMAP_JET)
		median_depth = np.transpose(np.nanmedian(avg_depth, axis=0))


		# From the median_depth get the objects that exist in the scene
		obstacles = util.get_obstacles(median_depth, depth_thresh)

		# Prune the obsticals to only include valid
		pruned_obstacles, pruned_medain = util.get_pruned_obstacles(obstacles, median_depth)
		
		# Find the largest gap in the image.
		largest_gap, gap_depth = util.find_largest_gap(pruned_obstacles, pruned_medain)
		
		# TODO: Add check here for if no gap exists! 
		car_pos = [340, 0]
		left_obs = [largest_gap[0], gap_depth[0]]
		right_obs = [largest_gap[1], gap_depth[1]]
		
		vec_left_obs = [largest_gap[0] - car_pos[0], gap_depth[0]]
		vec_right_obs = [largest_gap[1] - car_pos[0] , gap_depth[1]]

		# 1) Get the left line length
		left_len = math.sqrt((car_pos[0] - left_obs[0])**2 + (car_pos[1] - left_obs[1])**2)
		# 2) Get the right line length
		right_len = math.sqrt((car_pos[0] - right_obs[0])**2 + (car_pos[1] - right_obs[1])**2)

		# 3) Get the line length between the two obsticals
		middle_len = math.sqrt((right_obs[0] - left_obs[0])**2 + (right_obs[1] - left_obs[1])**2)
		
		# Calculate left and right angles! 
		vec_center = [0, 1]
		center_len = 1

		left_dot_prod = vec_left_obs[0] * vec_center[0] + vec_left_obs[1] * vec_center[1]
		right_dot_prod = vec_right_obs[0] * vec_center[0] + vec_right_obs[1] * vec_center[1]

		left_cos_theta = float(left_dot_prod)/(left_len * center_len)
		right_cos_theta = float(right_dot_prod)/(right_len * center_len)

		rad_left = math.acos(left_cos_theta)
		rad_right = math.acos(right_cos_theta)

		# TODO: I might need to check here to see if one should be negative! 
		ang_left = math.degrees(rad_left)
		ang_right = math.degrees(rad_right)

		
		# TODO: Calculate the gap angle! Given from paper
		cos_added = math.cos(rad_left + rad_right)
		numerator = left_len + right_len * cos_added
		denominator = math.sqrt(left_len**2 + right_len**2 + 2 * left_len * right_len * cos_added)
		rad_gap = math.acos(numerator/denominator) - rad_left
		ang_gap = math.degrees(rad_gap)
		print(ang_gap)

finally:
    pipeline.stop()
