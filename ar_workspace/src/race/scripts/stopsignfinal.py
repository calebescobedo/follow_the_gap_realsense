#!/usr/bin/env python

#Scott Scheraga 12/8/2019
#https://pysource.com/2018/12/29/real-time-shape-detection-opencv-with-python-3/
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import time
from matplotlib import pyplot as plt #for gradient stuff

cap = cv2.VideoCapture(0)
colorchooser=6;
# 1- color sliders
# 2- red
# 3- blue
# 4- green
# 5- yellow
# 6- wide detection for gradient filter - BEST!

if colorchooser==1:
   cv2.namedWindow("Trackbars")
   cv2.createTrackbar("L-H", "Trackbars", 0, 180, nothing)
   cv2.createTrackbar("L-S", "Trackbars", 66, 255, nothing)
   cv2.createTrackbar("L-V", "Trackbars", 134, 255, nothing)
   cv2.createTrackbar("U-H", "Trackbars", 180, 180, nothing)
   cv2.createTrackbar("U-S", "Trackbars", 255, 255, nothing)
   cv2.createTrackbar("U-V", "Trackbars", 243, 255, nothing)

font = cv2.FONT_HERSHEY_COMPLEX
timelimit=0.5
timestampsize = 17
timestamp = np.ones(timestampsize)
timestamp = timestamp*99

currenttimestamp = 0
stopcounter=0
octocount = 0
timetotal=99


def stop_sign(data):
    #_, frame = cap.read()
    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    bridge = CvBridge()
    #dtype, n_channels = br.encoding_as_cvtype2('8UC3')
    frame = bridge.imgmsg_to_cv2(data)
    frame = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
    #red = frame[:,:,0]
    #print('type', red.shape)
    #print('type', type(frame))
    #green = frame[:,:,1]
    #blue = frame[:,:,2]
    #frame[0] = blue
    #frame[1] = green
    #frame[2] = red
    #frame2 = np.asarray([[blue],[green], [red]])
    #frame2 = frame2.reshape((480,640,3))
    #print('shape', frame2.shape)
    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

    hsv = cv2.Laplacian(hsv,cv2.CV_64F) #gradient filter
    laplacian=hsv
    #print(time.time())
    

    if colorchooser==1:
        l_h = cv2.getTrackbarPos("L-H", "Trackbars")
        l_s = cv2.getTrackbarPos("L-S", "Trackbars")
        l_v = cv2.getTrackbarPos("L-V", "Trackbars")
        u_h = cv2.getTrackbarPos("U-H", "Trackbars")
        u_s = cv2.getTrackbarPos("U-S", "Trackbars")
        u_v = cv2.getTrackbarPos("U-V", "Trackbars")

        lower = np.array([l_h, l_s, l_v])
        upper = np.array([u_h, u_s, u_v])

    elif colorchooser==2:
#red
        lower = np.array([0, 66, 134])
        upper = np.array([20, 255, 245])

    elif colorchooser==3:
#blue
        lower = np.array([38, 86, 0])
        upper = np.array([121, 255, 255])

    elif colorchooser==4:
#green
        lower = np.array([88, 54, 44])
        upper = np.array([146, 255, 76])

    elif colorchooser==5:
#yellow
        lower = np.array([41, 54, 44])
        upper = np.array([75, 255, 76])
    elif colorchooser==6:
#yellow
        lower = np.array([0, 0, 0])
        upper = np.array([255, 255, 255])


    mask = cv2.inRange(hsv, lower, upper)

    kernel = np.ones((5, 5), np.uint8)
   # kernel = np.ones((3, 3), np.uint8)
    mask = cv2.erode(mask, kernel)
    mask = cv2.dilate(mask,kernel)

  
    
    # Contours detection
    if int(cv2.__version__[0]) > 3:
        # Opencv 4.x.x
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    else:
        # Opencv 3.x.x
        _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    for cnt in contours:
        cnt = cv2.convexHull(cnt) #convex hull
        area = cv2.contourArea(cnt)
        approx = cv2.approxPolyDP(cnt, 0.02*cv2.arcLength(cnt, True), True)
        x = approx.ravel()[0]
        y = approx.ravel()[1]

        if area > 1000: #400 was default
            cv2.drawContours(frame, [approx], 0, (0,159,0), 5)
	    # cv2.drawContours(frame, [approx], -1, (0, 255, 0), 3)
            #if len(approx) == 3 and cv2.isContourConvex(cnt) ==True:
           #     cv2.putText(frame, "Triangle", (x, y), font, 1, (0, 0, 0))
           # elif len(approx) == 4 and cv2.isContourConvex(cnt) ==True:
            #    cv2.putText(frame, "Rectangle", (x, y), font, 1, (0, 0, 0))
            #elif len(approx) == 8: #and area > 700:
            if len(approx) == 8: #and area > 700:
                cv2.putText(frame, "Octagon", (x, y), font, 1, (0, 0, 255))
                #print("Octagon found!", octocount )
                octocount +=1
                timestamp[currenttimestamp]=time.time()
                
                for n in range(0,(timestampsize-1)):
                    timetotal += (timestamp[n+1]-timestamp[n])

                if timetotal<timelimit: 
                    print("Stop command sent!", stopcounter )
                    stopcounter +=1
		   #(Send stop command to Caleb's code here!)	
                   
                currenttimestamp += 1
        
                if currenttimestamp==timestampsize:
                	currenttimestamp=0;

            elif 10 < len(approx) < 20 and cv2.isContourConvex(cnt) ==True:
                cv2.putText(frame, "Circle", (x, y), font, 1, (0, 0, 0))


    cv2.imshow("Frame", frame)
   
    #cv2.imshow("Gray", gray)
    cv2.imshow("laplacian", laplacian)
    cv2.imshow("Mask", mask)


    #sobelx = cv2.Sobel(gray,cv2.CV_64F,1,0,ksize=5)
    #sobely = cv2.Sobel(gray,cv2.CV_64F,0,1,ksize=5)
    
    #cv2.imshow("sobel_x", sobelx)
    #cv2.imshow("sobel_y", sobely)



    key = cv2.waitKey(1)

cap.release()
cv2.destroyAllWindows()

if __name__=='__main__':
    try: 
        rospy.init_node('stop_sign', anonymous=True)
        rospy.Subscriber("/camera/color/image_raw", Image, stop_sign)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
