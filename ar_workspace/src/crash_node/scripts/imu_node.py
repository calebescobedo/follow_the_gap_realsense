#!/usr/bin/env python
from Phidget22.Phidget import *
from Phidget22.Devices.Accelerometer import *
import rospy
from std_msgs.msg import Bool

def imu_talker():

    # Create new phidget22 accelerometer object
    accelerometer = Accelerometer()

    # Check if imu can be found by OS
    accelerometer.openWaitForAttachment(1000)

    try:
        pub = rospy.Publisher('crashed', Bool, queue_size=10)
        rospy.init_node('crash_detector', anonymous=True)
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            
            # Get current acceleration
            acceleration = accelerometer.getAcceleration()

            # Check if y acceleration has a large negative value in gs
            if acceleration[1] < -0.2:
                
                # In this case we have probably crashed
                crashed = True

            else:
                
                # In this case we probably didn't crash
                crashed = False

            pub.publish(crashed)
            rate.sleep()
    except rospy.ROSInterruptException:
        accelerometer.close()


if __name__ == '__main__':
    imu_talker()
















