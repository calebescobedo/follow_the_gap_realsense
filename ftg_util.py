
import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt 
import matplotlib.lines as mlines
import matplotlib.patches as mpatches

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

def plot_circles(circles, plt, ax, fig, x_vals, y_vals, ang_gap):
    # Get the X values in order to plot these points
    ax.clear()
    plt.xlim(0, 10000)
    plt.ylim(0, 6000)
    plt.gca().set_aspect('equal', adjustable='box')

    for c in circles:
        ax.add_artist(c)
    if not x_vals == []:
        first_x  = (x_vals[0]/640.0 * 10000)
        second_x = (x_vals[1]/640.0 * 10000)

        #line between both objects
        l = mlines.Line2D([first_x, second_x], y_vals)
        ax.add_line(l)

        #line from realsense to left object
        l = mlines.Line2D([first_x, 5000], [y_vals[0], 0])
        ax.add_line(l)

        #line from realsense to right object
        l = mlines.Line2D([second_x, 5000], [y_vals[1], 0])
        ax.add_line(l)

        # draw a line going through the center of the imag 
        l = mlines.Line2D([5000, 5000], [10000, 0])
        ax.add_line(l)


        #draw the arc for the circle at the bottom of the screen
        # TODO: I think the left and right angles should be switched.. they are wrong
        xy = [5000, 0]
        #arc = mpatches.Arc(xy, 1000, 1000, 90.0, 0, ang_right)
        #ax.add_patch(arc)
        arc = mpatches.Arc(xy, 1000, 1000, 90.0, 0, ang_gap)
        ax.add_patch(arc)

    fig.canvas.draw()


def get_obstacle_circles(depth_colormap, median_depth, obstacles):
    point_median_list = []
    circles = []
    obj_size_thresh = 20

    for row in obstacles:
        median = np.nanmedian(median_depth[row[0]:row[1]])
        obj_size = len(range(row[0],row[1]))
        if not median == 0 and obj_size > 20 and median < 5000:
            for x in range(0, obj_size):
                point_median_list.append(median)
            radius = (row[1] - row[0])/2
            middle = int((row[0] + radius)/640.0 * 10000)
            circles.append(plt.Circle((middle ,median),radius/640.0 * 10000,color='g'))
            #TODO: Move this rectagle drawing to it's own function. It's hidden in a bad way
            depth_colormap = cv.rectangle(depth_colormap, (row[0], 200), (row[1], 275), (0, 0 ,255), 2)		

    return circles


def get_pruned_obstacles(obstacles, median_depth):
    pruned_obstacles = []
    pruned_medain = []
    
    for row in obstacles:
        median = np.nanmedian(median_depth[row[0]:row[1]])
        obj_size = len(range(row[0],row[1]))
        if not median == 0 and obj_size > 20 and median < 5000:
            pruned_obstacles.append(row)
            pruned_medain.append(median)

    return pruned_obstacles, pruned_medain

def find_largest_gap(pruned_obstacles, pruned_medain):
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
    
    return largest_gap, gap_depth
			


# Might use later for getting only the 3 closest objects

#if len(obstacles) > 4:
		#	avg_vals = [np.nanmean(avg_depth[0,row]) for row in obstacles]
		#	avg_np = np.array(avg_vals)
		#	obs_np = np.array(obstacles)
		#	minimum_index = avg_np.argsort()[0:3]	
		#	obstacles = list(obs_np[minimum_index])	