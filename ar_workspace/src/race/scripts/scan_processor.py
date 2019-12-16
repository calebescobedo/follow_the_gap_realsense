#!/usr/bin/env python 
import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import ProcessedScan

# The front of the lidar is 0 degrees, specify the angle left and right that you want
# The reading will be given back from left to right
DEG_RANGE = 45.0
RAD_RANGE = math.pi/4


def rad_to_deg(rad):
    return rad * 180.0/math.pi

#the range array is 0-1439
def callback(data):
    # Create a ProcessedScan publisher
    # TODO: I need to check what the correct queue size and Rate should be
    pub = rospy.Publisher('processed_scan', ProcessedScan, queue_size=1)
    msg = ProcessedScan()

    rad_increment = data.angle_increment
    deg_increment = rad_to_deg(rad_increment)
    num_samples = int(DEG_RANGE/deg_increment)

    ranges = list(data.ranges)
    num_vals = len(ranges)

    left_vals = ranges[0:num_samples]
    left_vals.reverse()
    
    right_vals = ranges[num_vals-num_samples:num_vals]
    right_vals.reverse()
    
    scan = left_vals + right_vals

    # Package message to publish
    msg.deg_start = DEG_RANGE
    msg.deg_increment = deg_increment
    msg.rad_increment = rad_increment
    msg.measurements = scan 
    
    bearings = []
    cur_bearing = RAD_RANGE
    for val in scan:
        bearings.append(math.degrees(cur_bearing))
        cur_bearing = cur_bearing - rad_increment

    msg.bearings = bearings


    print(scan[0])
    print(scan[len(scan)/2])
    print(scan[len(scan)-1])
    #TODO: add a refresh rate to match this value
    pub.publish(msg)
    

def listener():
    rospy.init_node('scan_process_node', anonymous=True)
    rospy.Subscriber("scan", LaserScan, callback)
    
    rospy.spin()


if __name__ == '__main__':
    listener()
