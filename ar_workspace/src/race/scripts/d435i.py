#!/usr/bin/env python
import numpy as np

import rospy
from std_msgs.msg import Float32
from random import randint


LPIXOFF      = 35
RPIXOFF      = 35

VERTICAL_OFF = -45 # 40 original

def getAvg(d, x, y):
    sum = 0
    count = 0
    for i in range(0,3):
	for j in range(0, 3):
		if not d[x + i - 1][y + j - 1] == 0:
		    sum = sum + d[x+i-1][y+j-1]
		    count = count + 1
    if count == 0:
	return 0
    return float(sum)/float(count)

# Find mode in a series of observations
def get_mode(arr, tolerance):
    arr.sort()
    mode = arr[0]
    mode_count = 0

    curr_count = 0
    curr_element = mode
    for x in arr:
        if abs(x - curr_element) <= tolerance:
            curr_count = curr_count + 1
            if curr_element == mode:
                mode_count = mode_count + 1
        else:
            curr_element = x
            curr_count = 1

        if curr_count > mode_count:
            mode = curr_element
            mode_count = curr_count
    return mode

# get stable value from samples
def filterDistances(samples, tolerance):
    mode = get_mode(samples, tolerance)
    diff = abs(samples - mode)
    idx = np.where(diff <= tolerance)
    return np.mean(samples[idx])

def measureDistances(depth_frame, HEIGHT, WIDTH):
    width  = WIDTH
    height = HEIGHT
    dc = getAvg(depth_frame, int(height/2) + VERTICAL_OFF, int(width/2))
    dl = getAvg(depth_frame, int(height/2) + VERTICAL_OFF, LPIXOFF)
    dr = getAvg(depth_frame, int(height/2) + VERTICAL_OFF, width - RPIXOFF)

    return dc, dl, dr
