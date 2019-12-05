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
		object_circles = util.get_obstacle_circles(depth_colormap, median_depth, obstacles)
		





		# TODO: prune the obsticals to only include valid
		pruned_obstacles = []
		pruned_medain = []
		for row in obstacles:
			median = np.nanmedian(median_depth[row[0]:row[1]])
			obj_size = len(range(row[0],row[1]))
			if not median == 0 and obj_size > 20 and median < 4000:
				pruned_obstacles.append(row)
				pruned_medain.append(median)

		# TODO: I need to get the largest gap in the image.
		largest_gap = []
		gap_depth = []
		prev_end = 0


		for idx, [start, end] in enumerate(pruned_obstacles):
			if idx == 0:
				largest_gap = [0, start]
				gap_depth = [pruned_medain[idx], pruned_medain[idx]]
			else:
				if start - prev_end > largest_gap[1] - largest_gap[0]:
					largest_gap = [prev_end, start]
					gap_depth = [pruned_medain[idx-1], pruned_medain[idx]]
				if idx == len(pruned_obstacles) - 1:
					if 640 - end > largest_gap[1] - largest_gap[0]:
						largest_gap = [end, 640]
						gap_depth = [pruned_medain[idx], pruned_medain[idx]]
			prev_end = end
		if not largest_gap == []:
			depth_colormap = cv.rectangle(depth_colormap, (largest_gap[0], 200), (largest_gap[1], 275), (0, 255 ,0), 2)		

		util.plot_circles(object_circles, plt, ax, fig, largest_gap, gap_depth)


		# Show images
		real_sense_depth = "real_sense_depth"
		real_sense_color = "real_sense_color"
		cv.namedWindow(real_sense_color)
		cv.namedWindow(real_sense_depth)

		cv.moveWindow(real_sense_color, 800, 0)
		cv.moveWindow(real_sense_depth, 800, 500)

		cv.imshow(real_sense_depth, depth_colormap)
		cv.imshow(real_sense_color, color_image)
		cv.waitKey(1)

finally:
    pipeline.stop()
