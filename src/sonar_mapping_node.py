#!/usr/bin/env python

import rospy
import cv2 as cv
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import math
from skimage.draw import line_aa

def createImageCurrentScan(data):
    '''
    max_range = 0
    for range in data.ranges:
        if range > max_range and range < 10000:
            max_range = range
    print(max_range)
    '''



    # Array containing current scan data
    sonarDisplayArray = np.zeros((1500,1000), np.float32)
    positionManta = [1499,499]
    sonarDisplayArray[(positionManta[0]-50):(positionManta[0]),(positionManta[1]-50):(positionManta[1]+50)] = 1
    angle_increment = data.angle_increment
    current_angle = data.angle_min
    while current_angle <= data.angle_min:
        for index, range in enumerate(data.ranges):
            if range < 10000:
                # Projecting point from the sonar onto a 2D-plane given range and angle of current ping
                scan_width = math.sin(current_angle)*range
                scan_height = math.sqrt(pow(range,2) - pow(scan_width,2))
                sonarDisplayArray[positionManta[0] - int(scan_height*100),positionManta[1] - (scan_width*100)] = 1

                # Colors squares between surface and solid gray to represent area without obstacles
                #y_point, x_point = [positionManta[0],positionManta[1]]
                #while (y_point > abs(positionManta[0]-scan_height)) and (x_point > abs(positionManta[1]-scan_width)):


            # Each scan containts X-amount of pings with an increment in angle between the pings which have to be updated
            current_angle+=angle_increment 
    
        #Visualising sonar image
        cv.imshow("sonarDisplay",sonarDisplayArray)
        cv.waitKey(50)
           
    

    
    



def sonarCallback(data):
    createImageCurrentScan(data)

def mapping():
    rospy.init_node('sonar_mapping',anonymous=True)
    rospy.Subscriber('manta/sonar',LaserScan,sonarCallback)
    rospy.spin()

if __name__ == '__main__':
    mapping()



