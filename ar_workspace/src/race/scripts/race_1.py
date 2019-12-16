#!/usr/bin/env python
#import pyrealsense2 as rs
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time

import rospy
from std_msgs.msg import Float32
from random import randint

from d435i import measureDistances
from d435i import filterDistances

from ros_pololu_servo.msg import MotorCommand

# realsense defs
WIDTH        = 640 #424
HEIGHT       = 480 #240
FPS          = 60
DIST_EPS     = 0.05
DEPTH_FRAMES = 30
LPIXOFF      = 35
RPIXOFF      = 5

DEBUG        = 1

# STATE MACHINE CODE
WALL_FOLLOW = 1 # state number
CORNERING   = 2 # state number
BACKOUT     = 3

# SLIDING WINDOW PARAMS
TOLERANCE = 0.1 
WINDOW_SIZE = 5

# WALL_FOLLOW PARAMS
MINIMUM_FRONT_DIST = 3 # For any dc value smaller than this, take a sharp turn (STATE: CORNERING)
STRAIGHT_THRESHOLD = 0.3 # As long as abs(dr - dl) is within this, no turning needed

WALL_FOLLOW_VELOCITY = 0.15
WALL_FOLLOW_TURN_ANGLE = 0.25

# SPECIAL CASE (IF NEEDED)
SPECIAL_CASE_THRESHOLD = 6.0

# CORNERING PARAMS
CORNERING_TURN_ANGLE    = 0.75 
CORNERING_THRESHOLD = 0.1
CORNERING_VELOCITY = 0.1

def actuate(ang, ang_speed, ang_acc, vel, vel_speed, vel_acc):
    #cmd.position: radians, -45 min (left), -45 max (right)
    #speed and accel 0 to 1
    cmd = MotorCommand()
    cmd.joint_name = "steering"
    #cmd.position = degToRad(ang)
    cmd.position = ang
    cmd.speed = ang_speed
    cmd.acceleration = ang_acc
    print("Setting angle to ", ang)
    pololu_pub.publish(cmd)

    #cmd.position: radians, 0 min, 45 max (1560)
    #speed and accel 0 to 1
    cmd.joint_name = "drive"
    #cmd.position = degToRad(vel)
    cmd.position = vel
    cmd.speed = vel_speed
    cmd.acceleration = vel_acc
    print("Setting speed to ", vel)
    pololu_pub.publish(cmd)

def wallFollow():
    global dl
    global dr
    global dc
    global state

    # for now just keep at a constant angle
    speed = WALL_FOLLOW_VELOCITY
    angle = WALL_FOLLOW_TURN_ANGLE
    
    #if abs(dl - dr) > SPECIAL_CASE_THRESHOLD:
    #    print("Special case")
    #    print("Continue in thee current Direction")
    #    angle = 0
    #    actuate(angle, 1, 0, speed, 1, 0)

    # normal tracks
    if dr - dl > STRAIGHT_THRESHOLD:
        print(dr - dl, STRAIGHT_THRESHOLD)
        print("Turning to right")
        angle = WALL_FOLLOW_TURN_ANGLE
    elif dl - dr > STRAIGHT_THRESHOLD:
        print(dl - dr, STRAIGHT_THRESHOLD)
        print("Turning to left")
        angle = -1 * WALL_FOLLOW_TURN_ANGLE
    else:
        print("No need to turn")
        angle = -0.05
    
    actuate(angle, 1, 0, speed, 1, 0)

def cornering():
    global state
    # definitely turn right
    
    global dl
    global dc
    global dr

    if dr - dl > CORNERING_THRESHOLD: 
        angle = CORNERING_TURN_ANGLE
        speed = CORNERING_VELOCITY
    elif dr - dl < CORNERING_THRESHOLD:
        angle = CORNERING_TURN_ANGLE # multiply by -1 if you want to turn left
        speed = CORNERING_VELOCITY

    
    actuate(angle, 1, 0, speed, 1, 0)

def controller(dl, dr, dc):
    global state

    if dc < MINIMUM_FRONT_DIST:
        print("STATE: CORNERING")
        state = CORNERING
    elif dc > MINIMUM_FRONT_DIST:
        print("STATE: WALL FOLLOW")
        state = WALL_FOLLOW
    
    if state == WALL_FOLLOW:
        wallFollow()
    elif state == CORNERING:
        cornering()
    elif state == BACKOUT:
        backout()
        
def callbackDepth(depth_image):
    global bridge
    global depth

    print("Got image")
    depth = bridge.imgmsg_to_cv2(depth_image, depth_image.encoding)

    # GLOBALS

    # Initial State
    global state # WALL_FOLLOW=1, CORNERING=2
    
    # Distance vars
    global dl
    global dr 
    global dc

    global pololu_pub
    pololu_pub = rospy.Publisher('pololu/command', MotorCommand, queue_size=10)
    # GLOBALS END

    # Sliding Window Vars
    global idx_c   
    global idx_l
    global idx_r

    global dist_c_window 
    global dist_l_window
    global dist_r_window 

    rospy.loginfo("Publish and Subscribe: initialized");

    # compute dl, dr, dc for this frame
    dc, dl, dr = measureDistances(depth, HEIGHT, WIDTH)
    dc = float(dc)/1000.0
    dl = float(dl)/1000.0
    dr = float(dr)/1000.0

    if DEBUG:
        print("Sensor Readings:\n")
        print(str(dc) + " " + str(dl) + " " + str(dr))

    if not dc == 0:
	if idx_c < WINDOW_SIZE:
	    dist_c_window.append(dc)
	else:
	    dist_c_window[idx_c % WINDOW_SIZE] = dc
	idx_c = idx_c + 1

    #if not dl == 0:
    if idx_l < WINDOW_SIZE:
        dist_l_window.append(dl)
    else:
        dist_l_window[idx_l % WINDOW_SIZE] = dl
    idx_l = idx_l + 1

    #if not dr == 0:
    if idx_r < WINDOW_SIZE:
        dist_r_window.append(dr)
    else:
        dist_r_window[idx_r % WINDOW_SIZE] = dr
    idx_r = idx_r + 1

    if len(dist_c_window) > 0:
        dist_c = filterDistances(np.array(dist_c_window), DIST_EPS)
    else:
        dist_c = 0
    
    if len(dist_l_window) > 0:
        dist_l = filterDistances(np.array(dist_l_window), DIST_EPS)
    else:
        dist_l = 0

    if len(dist_r_window) > 0:
        dist_r = filterDistances(np.array(dist_r_window), DIST_EPS)
    else:
        dist_r = 0

    if(DEBUG):
        print("Filtered Values:\n")
        print(str(dist_c) + "  " + str(dist_l) + "  " + str(dist_r))


    ## CONTROL DECISION
    controller(dist_l, dist_r, dist_c)

def degToRad(ang):
    return ang*np.pi/180
    
    
if __name__=='__main__':
    try:
        #distance_publisher()
        rospy.init_node('race',anonymous=True)
        bridge = CvBridge()
        
        # Initial State
        state = WALL_FOLLOW
        
        # Distance vars
        dl = 0.0
        dr = 0.0
        dc = 0.0

        idx_c = 0
        idx_l = 0
        idx_r = 0
        dist_c_window = []
        dist_l_window = []
        dist_r_window = []
        print("Running")
        rospy.Subscriber("/camera/depth/image_rect_raw", Image, callback= callbackDepth)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
