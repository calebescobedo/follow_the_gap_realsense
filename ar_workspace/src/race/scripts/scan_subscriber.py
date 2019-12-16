#/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan 

def callback(data):
    
    print('hello')
    print(data.ranges)

def listener():

    rospy.init_node('listener')
    rospy.Subscriber("scan",LaserScan, callback)
    rospy.spin()
 
if __name__ == '__main__':
    listener()
