import rospy
import numpy as np
import cv2
#from cv2 import cv
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



# Takes inn two numpy arrays y and x, and finds the best line fit for all points, writes intersection and slope to calibration.txt
def fit_line_least_squares(self,y,x):
        # x and y is numpy arrays
        meanX = float(np.sum(x))/float(len(x))
        meanY = float(np.sum(y))/float(len(y))
        # Line equation: y = ax + b
        a_upper = 0
        a_lower = 0
        for i in range(len(x)):
            a_upper += ((float(x[i])-meanX)*(float(y[i])-meanY))
            a_lower += pow((float(x[i])-meanX),2)
        a = a_upper/a_lower
        b = meanY - a*meanX
        # Write fitted line parameters to file
        f = open('calibration.txt','w')
        f.write('a: ' + str(a) + '\nb: ' + str(b) + '\n')

# Converts a CompressedImage to an openCV image
def convert_to_openCV(image_msg):
    bridge = CvBridge()
    try: 
        image = bridge.imgmsg_to_cv2(image_msg, "bgr8")
        cv2.imshow('Display',image)
        cv2.waitKey(1)
        return image
    except:
        return []

