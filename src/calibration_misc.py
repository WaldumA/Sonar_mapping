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
    except:
        return []

# Switcher for handling user input
def input_switcher(user_input):
    Switcher = {
        0: 'Save',
        1: 'Calibrate',
    }
    task = Switcher.get(user_input,'skip')
    if task == 'skip':
        return 'Skip'
    elif task == 'Save':
        return 'Save'
    elif task == 'Calibrate':
        return 'Calibrate'

# Return bearing, range and pixel width of current msg
def Save_current_frame(sonar_data, image_data):
    # Extracting necesarry data from msgs
    angle_increment = sonar_data.angle_increment
    current_angle = sonar_data.angle_min
    scan_ranges = sonar_data.ranges
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(image_data, "bgr8")
    # Appending all promising scans to a list as tuples [range_of_scan, bearing_from_manta]
    possible_scans = get_promising_sonar_scans(angle_increment,current_angle,scan_ranges)

    # Calculating average range and bearing
    sonar_range, sonar_bearing = calculate_best_sonar_data(possible_scans)

    # Finds current pixel width of pole
    pixel_width = get_pixel_width(image)

    return pixel_width, sonar_bearing, sonar_range 




# Calibrate
def calibrate_lines():
    pass

# Writing fitted variables to a .txt file
def write_to_file():
    pass

# Returns all depth scans betlow a certain threshold with bearing
def get_promising_sonar_scans(angle_increment,current_angle,scan_ranges):
    possible_scans = []
    counter = 0
    for depth in scan_ranges:
        if depth < 10:
            possible_scans.append(tuple([depth,current_angle]))
            counter += 1
        current_angle += angle_increment
    return possible_scans

# Filters a colored pole and detects its bounding box
def get_pixel_width(image):
    # Transforms the image to HSV which is a color space easier to filter 
    image_hsv=cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # In the HSV colorspace RED is represented both from 170-180 and 0-10
    # 0-10
    lower_red = np.array([0,50,50])
    upper_red = np.array([10,255,255])
    mask0 = cv2.inRange(image_hsv, lower_red, upper_red)
    # 170-180
    lower_red = np.array([170,50,50])
    upper_red = np.array([180,255,255])
    mask1 = cv2.inRange(image_hsv, lower_red, upper_red)

    # Combining upper and lower values
    mask = mask0 + mask1
    
    # Finds contours
    _,contours,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    area = 0
    bbox = -1  
    pixel_width = -1
    for contour in contours:
        if cv2.contourArea(contour) > area:
            area = cv2.contourArea(contour)
            bbox = cv2.boundingRect(contour)
       
    if bbox != -1:
        pixel_width = bbox[0] + int((bbox[0]+bbox[2])/2)
        ''' 
        # Visualising bounding box for troubleshooting purposes
        cv2.rectangle(image,(bbox[0],bbox[1]),(bbox[0]+bbox[2],bbox[1]+bbox[3]),(0,255,0), 3)  
        cv2.imshow('Display',image)
        cv2.waitKey()
        '''
    return pixel_width
    
def cal