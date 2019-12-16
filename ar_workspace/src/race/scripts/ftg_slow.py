#!/usr/bin/env python
import math
import rospy
import numpy as np
import copy
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from ros_pololu_servo.msg import MotorCommand
import matplotlib.pyplot as plt
import matplotlib

DEPTH_THRESH = 3
VELOCITY = -0.12

Clockwise = False


global pololu_pub
pololu_pub = rospy.Publisher('pololu/command', MotorCommand, queue_size=10)
global prev_angle
global pub
pub = rospy.Publisher('ftgAngle', Float64, queue_size=10)

def test(data):
    pub_gap = rospy.Publisher('ftg', LaserScan, queue_size=1)
    pub_no_gap = rospy.Publisher('noGap', Bool, queue_size=10)

    # It would be good here to package this angle information into the message
    left_ang = np.rad2deg(data.angle_max)
    right_ang = np.rad2deg(data.angle_max)
    inc_ang = np.rad2deg(data.angle_increment)

    depths = data.ranges
    depth_thresh = DEPTH_THRESH
    depths = list(depths)

    for idx, depth in enumerate(depths):
        if depth > depth_thresh:
            depths[idx] = 0

    c_depths = copy.deepcopy(depths)

    start = 0
    num_zeros = 0
    in_gap = False
    cur_zeros = 0
    cur_start = 0

    for idx, depth in enumerate(depths):
        if not depth == 0:
            depths[idx] = 1
            cur_zeros = 0
            in_gap = False
        else:
            if not in_gap:
                cur_start = idx
                cur_zeros = 0
                in_gap = True
            depths[idx] = 0.5
            cur_zeros += 1
            if cur_zeros > num_zeros:
                num_zeros = cur_zeros
                start = cur_start
    middle = start + num_zeros/2
    ang = 45.0 - inc_ang * middle
    
    c_depths_2 = copy.deepcopy(depths)
    c_start = 0
    in_gap = False
    cur_zeros = 0
    cur_start = 0
    window_size = 20
    sum_max = 0
    cur_sum = 0
    mid_idx = 0
    go_far = False

    section_len = 100
 
    # check if no gap big enough exists
    if num_zeros < 150:
        go_far = True
        for idx, depth in enumerate(c_depths_2):
            if idx - section_len < len(c_depths_2):
                cur_sum = sum(c_depths_2[idx:idx + section_len - 1])
                if cur_sum < sum_max:
                    sum_max = cur_sum
                    mid_idx = idx + int(section_len/2)
        print("Go at the furthest object!")
    
    no_pub = False
    if go_far:
        frac = mid_idx/float(len(c_depths))
        frac = frac * 90.0
        ang = -(frac - 45.0)
        #ang = ang - (0.2 * ang)
        if ang < 0:
            if Clockwise:
                ang = 45.0
            else:
                ang = -45.0
        else:
            if Clockwise:
                ang = -45.0
            else: 
                ang = 45.0
        real_ang = ang
        
    print('Angle', ang, 'start', start, 'num zeros', num_zeros) 


    msg = MotorCommand()
    msg.joint_name = "drive"
    msg.acceleration = 0
    msg.speed = 1
    msg.position = VELOCITY
    pololu_pub.publish(msg)

	
    msg.joint_name = "steering"
    ang = (((ang + 45.0)/90.0) * 1.4) - 0.9
    msg.acceleration = 0
    msg.speed = 1
    msg.position = ang
    if not no_pub:
        pololu_pub.publish(msg)

    """
        no_gap_msg = Bool()
    if go_far:
        no_gap_msg.data = True
    else:
        no_gap_msg.data = False
    pub_no_gap.publish(no_gap_msg)

    print("ANGLE!!!!!", real_ang)
    msg.ranges = gap_depths
    pub_gap.publish(msg)


    # Publish the method for MAX to process and send to the steering
    pub_ang = Float64()
    pub_ang.data = real_ang
    pub.publish(pub_ang)
    """
def listen():
    rospy.init_node('ftg', anonymous=False)
    rospy.Subscriber("scan", LaserScan, test)
    rospy.spin()

def publish_steering_angle(angle):
    cmd = MotorCommand()
    print("Angle: " + str(angle))

    ftg_msg = Float64()
    ftg_msg.data = angle

    pub.publish(ftg_msg)

if __name__ == '__main__':
    try:
        prev_angle = 0
        listen()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
