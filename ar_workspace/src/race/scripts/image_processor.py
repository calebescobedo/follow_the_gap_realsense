#!/usr/bin/env python 
import rospy
import math
import numpy as np
import matplotlib.pyplot as plt  
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ros_pololu_servo.msg import MotorCommand

depth_lower_bound = 275
depth_image_height = 75
depth_thresh = 100
depth_upper_bound = depth_lower_bound - depth_image_height 
sample_measurements = {}
num_samples = 0

def plot_trajectories(sample_measurements):
    
    angles = sample_measurements.keys()
    depths = sample_measurements.items()
    colors = angles
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='polar')
    c = ax.scatter(angles, depths, c=colors, cmap='hsv', alpha=0.75)
    plt.show()

def get_pruned_obstacles(obstacles, median_depth): 
    pruned_obstacles = []
    pruned_median = []

    for row in obstacles:
	median = np.nanmedian(median_depth[row[0]:row[1]])
	obj_size = len(range(row[0], row[1]))
	if not median == 0 and obj_size > 20 and median < 5000:
	    pruned_obstacles.append(row)
	    pruned_median.append(median) 

    return pruned_obstacles, pruned_median

def find_largest_gap(pruned_obstacles, pruned_median):
    
    largest_gap = []
    gap_depth = [] 
    prev_end = 0
    
    for idx, [start, end] in enumerate(pruned_obstacles):
	if idx == 0:
	    largest_gap = [0, start]
	    gap_depth = [pruned_median[idx], pruned_median[idx]]
	else:
	    if start - prev_end > largest_gap[1] - largest_gap[0]:
		largest_gap = [prev_end, start]
		gap_depth = [pruned_median[idx-1], pruned_median[idx]]
	    if idx == len(pruned_obstacles) - 1:
		if 640 - end > largest_gap[1] - largest_gap[0]:
		    largest_gap = [end, 640]
		    gap_depth = [pruned_median[idx], pruned_median[idx]]
	prev_end = end 
    
    return largest_gap, gap_depth 

def callback(data):

    pub = rospy.Publisher('motor_command', MotorCommand, queue_size=1)
    msg = MotorCommand()     

    bridge = CvBridge()
    depth = bridge.imgmsg_to_cv2(data, data.encoding)
    height = data.height
    width = data.width

    depth_image_cut = depth[depth_upper_bound:depth_lower_bound, :]
    median_depth = np.transpose(np.nanmedian(depth_image_cut, axis=0))

    obstacles = get_obstacles(median_depth, depth_thresh) 
    pruned_obstacles, pruned_median = get_pruned_obstacles(obstacles, median_depth) 
    largest_gap, gap_depth = find_largest_gap(pruned_obstacles, pruned_median) 

    car_pos = [340, 0]
    left_obs = [largest_gap[0], gap_depth[0]]
    right_obs = [largest_gap[1], gap_depth[1]]

    l_neg = False
    r_neg = False 

    if largest_gap[0] > 340:
	l_neg = True 
    if largest_gap[1] > 340:
	r_neg = True 

    vec_left_obs = [largest_gap[0] - car_pos[0], gap_depth[0]]
    vec_right_obs = [largest_gap[1] - car_pos[0], gap_depth[1]]

    # Get left, right and middle line lengths
    left_len = math.sqrt((car_pos[0] - left_obs[0])**2 + (car_pos[1] - left_obs[1])**2)
    right_len = math.sqrt((car_pos[0] - right_obs[0])**2 + (car_pos[1] - right_obs[1])**2)
    middle_len = math.sqrt((right_obs[0] - left_obs[0])**2 + (right_obs[1] - left_obs[1])**2)

    # Calculate left and right angles 
    vec_center = [0, 1]
    center_len = 1

    left_dot_prod = vec_left_obs[0] * vec_center[0] + vec_left_obs[1] * vec_center[1]
    right_dot_prod = vec_right_obs[0] * vec_center[0] + vec_right_obs[1] * vec_center[1]
    
    left_cos_theta = float(left_dot_prod)/(left_len * center_len)
    right_cos_theta = float(right_dot_prod)/(right_len * center_len)

    rad_left = math.acos(left_cos_theta)     
    rad_right = math.acos(right_cos_theta)     

    ang_left = math.degrees(rad_left)
    if l_neg:
	ang_left *= -1
    print('left angle: ' + str(ang_left))
    ang_right = math.degrees(rad_right)
    if r_neg:
	ang_right *= -1 
    print('right angle: ' + str(ang_right))

    # Simpler method of calculating the depth angle 
    avg_ang = (ang_left + ang_right)/2
    global num_samples
    global sample_measurements
    if num_samples < 50:
        sample_measurements[avg_ang] = gap_depth
	num_samples += 1

    # Calculate the gap angle (precise method)  
    # cos_added = math.cos(rad_left + rad_right)
    # numer = left_len + right_len * cos_added 
    # denom = math.sqrt(left_len**2 + right_len**2 + 2*left_len*right_len*cos_added)
    # rad_gap = math.acos(numer/denom) - rad_left
    # ang_gap = math.degrees(rad_gap)  
    
    pololu_pub = rospy.Publisher('pololu/command', MotorCommand, queue_size=10)
    # print('depth', type(depth))
    # print("height", height, 'width', width)
    
    msg.joint_name = 'steering'
    #ang_mapped = math.radians(-avg_ang)
    ang_mapped = (((avg_ang + 30)/60) * 1.4) - 0.7
    msg.position = ang_mapped

    pololu_pub.publish(msg) 
    print('angle output: ' + str(avg_ang)) 

def listener():
    
    rospy.init_node('scan_process_node', anonymous=True)
    rospy.Subscriber("/camera/depth/image_rect_raw", Image, callback)
    rospy.spin()
    
    
def get_obstacles(median_depth, depth_thresh): 
    prev_pixel = median_depth[0]
    start = 0
    end = 0 
    obstacles = [] 

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
	   
if __name__ == '__main__':
    
    listener()
    plot_trajectories(sample_measurements)
