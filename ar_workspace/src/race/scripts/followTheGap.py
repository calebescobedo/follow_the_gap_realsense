#!/usr/bin/env python

#--------
#This is an implementation of the Follow The Gap method for use in CSCI 7800
#The Lidar input required for this code is a list of bearing-distance objects, containing a variable called bearing and a variable called distance.
#I've only tested with bearing in degrees, from -180 to 180, with 0 being in front of the vehicle. It should work similarly in radians (-pi to +pi, 0 in front) as long as MODE is set to 'rad' (or really anything besides 'deg').
#Currently, this code uses linear restrictions in turn angle, under the assumption that the resultant path should be long enough to avoid restrictions within a reasonably narrow search band. If desired, the limit angles can be varied based on current angular momentum.
#--------

from copy import deepcopy
import math
import rospy
from race import BearingDistPlane
from std_msgs.msg import float32

INFINITY = float('inf')
MODE = 'deg'

def setObstacleHorizon(readings, maxDist):
    readCopy = deepcopy(readings)
    for i in readCopy:
        i.dist = i.dist if i.dist<maxDist else INFINITY
    return(readCopy)

def findGaps(readings):
    gapList = []
    start = None
    lastObst = None
    gap = False
    n = len(readings)
    for i in range(n):
        if readings[i].dist < INFINITY:
            start = i
            break
    if start == None:
        return(-1)
    for j in range(n):
        i = (j+start)%n
        if gap:
            if readings[i].dist < INFINITY:
                gapList = gapList + [[lastObst[0], readings[i].bearing, lastObst[1], readings[i].dist]]
                gap = False
                lastObst = [readings[i].bearing, readings[i].dist]
        else:
            if readings[i].dist == INFINITY:
                gap = True
            else:
                lastObst = [readings[i].bearing, readings[i].dist]
    if gap:
        gapList = gapList + [[lastObst[0], readings[start].bearing, lastObst[1], readings[start].dist]]
    return(gapList)

def findMaxGap(gaps, limits): #limits should be given as a two-item list of angles, as [left, right].
    if MODE == 'deg':
        maxBearing = 360
    else:
        maxBearing = math.pi*2
    gapsCopy = deepcopy(gaps)
    if limits[0] < limits[1]:
        for gap in gapsCopy:
            gap[0] = max(min(gap[0], limits[1]), limits[0])
            gap[1] = max(min(gap[1], limits[1]), limits[0])
        gapsCopy.sort(key = lambda a : a[1]-a[0], reverse = True)
    else:
        modLim = limits[0]-maxBearing
        for gap in gapsCopy:
            gap[0] = max(min(gap[0], limits[1]), modLim)%maxBearing
            gap[1] = max(min(gap[1], limits[1]), modLim)%maxBearing
        gapsCopy.sort(key = lambda a : a[1]-a[0]+maxBearing, reverse = True)
    return(gapsCopy[0])

def findGapAngle(gap):
    if gap[0] == gap[1]:
        return(None)
    maxBearing = math.pi*2
    if MODE == 'deg':
        maxBearing = 360
    center = ((gap[0]+gap[1]+maxBearing)/2)%maxBearing if gap[0] > gap[1] else (gap[0]+gap[1])/2
    g0 = abs(gap[0]-center)
    g1 = abs(gap[1]-center)
    if MODE == 'deg':
        gapAngle = math.degrees(math.acos((gap[2]+(gap[3]*math.cos(math.radians(g0+g1))))/(gap[2]**2+gap[3]**2+2*gap[2]*gap[3]*math.cos(math.radians(g0+g1)))**0.5))-g0
        return(center + gapAngle)
    else:
        gapAngle = math.acos((gap[2]+(gap[3]*math.cos(g0+g1)))/(gap[2]**2+gap[3]**2+2*gap[2]*gap[3]*math.cos(g0+g1))**0.5)-g0
        return(center + gapAngle)

def findGapAngleBasic(gap):
    if gap[0] == gap[1]:
        return(None)
    maxBearing = math.pi*2
    if MODE == 'deg':
        maxBearing = 360
    if gap[0] > gap[1]:
        return(((gap[0]+gap[1]+maxBearing)/2)%maxBearing)
    else:
        return((gap[0]+gap[1])/2)

def findClosestObstacleDist(readings, checkRange): #checkRange should be in format [minBearing, maxBearing]
    minDist = INFINITY
    for i in readings:
        if i.bearing > checkRange[0] and i.bearing < checkRange[1]:
            minDist = min(minDist, i.dist)
    return(minDist)
	
def followTheGap(readings, obstHorizon, limits, goal, alpha, basic_mode = False): #This is the business function; you really only need to use this one, as it calls all the others
    readingsWithHorizon = setObstacleHorizon(readings, obstHorizon)
    gaps = findGaps(readingsWithHorizon)
    if gaps == -1:
        return(goal)
    if len(gaps) == 0:
        return(None)
    maxGap = findMaxGap(gaps, limits)
    maxGapAngle = findGapAngleBasic(maxGap) if basic_mode else findGapAngle(maxGap)
    if maxGapAngle == None:
        return(None)
    minObstDist = findClosestObstacleDist(readings, limits)
    finalAngle = (maxGapAngle*(alpha/minObstDist) + goal)/((alpha/minObstDist)+1)
    return(finalAngle)

def listen():
    rospy.init_node('ftg', anonymous=False)
    rospy.Subscriber("lidar", BearingDistPlane, publish)
    rospy.spin()

def publish(data):
    pub = rospy.Publisher('ftgAngle', float32, queue_size=10)
    readings = processData(data)
    pub.publish(followTheGap(readings, 5, ((-45, 45)), 0, 0.1))

if __name__ == '__main__':
    try:
        listen()
    except rospy.ROSInterruptException:
        pass
