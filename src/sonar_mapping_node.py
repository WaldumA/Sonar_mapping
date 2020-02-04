#!/usr/bin/env python

import rospy
import cv2 as cv
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import math
import misc

def createImageCurrentScan(data):
    '''
    max_range = 0
    for range in data.ranges:
        if range > max_range and range < 10000:
            max_range = range
    print(max_range)
    '''

    # Global variables, into YAML file in the future
    WALL_WIDTH = 3
    # Array containing current scan data
    sonarDisplayArray = np.ones((1500,1000), np.float32)
    positionManta = [1498,499]
    angle_increment = data.angle_increment
    current_angle = data.angle_min
    while current_angle <= data.angle_min:
        for index, range in enumerate(data.ranges):
            if range < 10000:
                # Projecting point from the sonar onto a 2D-plane given range and angle of current ping
                scan_width = math.sin(current_angle)*range
                scan_height = math.sqrt(pow(range,2) - pow(scan_width,2))
                array_height = positionManta[0] - int(scan_height*100)
                array_width = positionManta[1] - (scan_width*100)
                

                # Colors squares between surface and solid gray to represent area without obstacles and black to represent solid walls
                rr, cc, val = misc.getLineBetweenPoints(array_height,array_width,positionManta[0],positionManta[1])
                sonarDisplayArray[rr,cc] = 0.8
                sonarDisplayArray[array_height-WALL_WIDTH:array_height+WALL_WIDTH,array_width-WALL_WIDTH:array_width+WALL_WIDTH] = 0
                sonarDisplayArray[(positionManta[0]-20):(positionManta[0]),(positionManta[1]-10):(positionManta[1]+20)] = 0
                #y_point, x_point = [positionManta[0],positionManta[1]]
                #while (y_point > abs(positionManta[0]-scan_height)) and (x_point > abs(positionManta[1]-scan_width)):


            # Each scan containts X-amount of pings with an increment in angle between the pings which have to be updated
            current_angle+=angle_increment 
    
        #Visualising sonar image
        cv.imshow("sonarDisplay",sonarDisplayArray)
        cv.waitKey(1)
           
    

    
    



def sonarCallback(data):
    createImageCurrentScan(data)

def mapping():
    rospy.init_node('sonar_mapping',anonymous=True)
    rospy.Subscriber('manta/sonar',LaserScan,sonarCallback)
    rospy.spin()

if __name__ == '__main__':
    mapping()



