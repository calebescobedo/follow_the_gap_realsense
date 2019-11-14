
import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt 









def setup_figure(plt):
    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(1,1,1)
    return fig, ax

def setup_realsense(rs):
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    return pipeline

def get_frames(pipeline):
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    return depth_frame, color_frame
    
def get_obstacles(median_depth, depth_thresh):
    prev_pixel = median_depth[0]
    start = 0
    end = 0
    obstacles = []

    # Get list of all objects in the image
    for idx, cur_pixel in enumerate(median_depth):

        if idx == len(median_depth) - 1:	
            obstacles.append([start, end])
        elif abs(prev_pixel - cur_pixel) < depth_thresh:
            end += 1
        else:
            obstacles.append([start, end])
            start = idx
            end = idx
        prev_pixel = cur_pixel

    return obstacles

def plot_circles(circles, plt, ax, fig):
    # Get the X values in order to plot these points
    ax.clear()
    plt.xlim(0, 10000)
    plt.ylim(0, 10000)
    plt.gca().set_aspect('equal', adjustable='box')

    for c in circles:
        ax.add_artist(c)
    fig.canvas.draw()

def get_obstacle_circles(depth_colormap, median_depth, obstacles):
    point_median_list = []
    circles = []

    for row in obstacles:
        median = np.nanmedian(median_depth[row[0]:row[1]])
        obj_size = len(range(row[0],row[1]))
        if not median == 0 and obj_size > 10:
            for x in range(0, obj_size):
                point_median_list.append(median)
            radius = (row[1] - row[0])/2
            middle = int((row[0] + radius)/700.0 * 10000)
            circles.append(plt.Circle((middle ,median),radius*10,color='g'))

        depth_colormap = cv.rectangle(depth_colormap, (row[0], 200), (row[1], 275), (0, 0 ,255), 2)		

    return circles




# Might use later for getting only the 3 closest objects

#if len(obstacles) > 4:
		#	avg_vals = [np.nanmean(avg_depth[0,row]) for row in obstacles]
		#	avg_np = np.array(avg_vals)
		#	obs_np = np.array(obstacles)
		#	minimum_index = avg_np.argsort()[0:3]	
		#	obstacles = list(obs_np[minimum_index])	