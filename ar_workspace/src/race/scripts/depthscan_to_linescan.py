#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan,Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

BRIDGE = CvBridge() # Bridge object to get depth image
DEPTH_SCALE = 0.0010000000474974513 # Conversion factor to convert depth to m
DEPTH_FOV = 90 # Horizontal field of view of depth camera in degrees
#LINE_POSITION = 0.6
LINE_POSITION = 0.45
RS_FPS = 30
MAX_RANGE = 8

def ls_publisher(line_scan,width):
	# Calculate the minimum and maximum angle in degrees
	min_max_angle_deg = DEPTH_FOV/2.
	
	# Convert min and max angles to radians
	min_max_angle_rad = np.deg2rad(min_max_angle_deg)
	
	# Calculate the angular resolution
	angle_increment = np.deg2rad(DEPTH_FOV)/width
	
	# Create laser scan message object
	msg = LaserScan()
	msg.angle_min = -min_max_angle_rad
	msg.angle_max = min_max_angle_rad
	msg.angle_increment = angle_increment
	msg.time_increment = 0
	msg.scan_time = 1./RS_FPS
	msg.range_min = 0.0
	msg.range_max = MAX_RANGE
	msg.ranges = line_scan
	msg.intensities = np.asanyarray([])
	pub = rospy.Publisher('/scan',LaserScan,queue_size=1)
	pub.publish(msg)	

def rs_callback(data):
	# Get depth image and convert to m
	depth = BRIDGE.imgmsg_to_cv2(data,data.encoding)*DEPTH_SCALE
	height = data.height
	width = data.width
	#depth[np.where(depth<0)] = 0
	#depth[np.where(depth>5)] = 5

	# Calculate index of line scan 
	line_index = int(LINE_POSITION*height)

	# Get line scan from depth array
	line_scan = depth[line_index,:]
	
	# Call function to publish line scan data
	ls_publisher(line_scan,width)

def rs_listener():
	rospy.init_node('depth_to_linescan')
	rospy.Subscriber('/camera/depth/image_rect_raw',Image,rs_callback)
	rospy.spin()

if __name__ == '__main__':
	
	try:
		rs_listener()
	except rospy.ROSInterruptException:
		pass 
