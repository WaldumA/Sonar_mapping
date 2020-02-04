#!/usr/bin/env python

import rospy
import cv2 as cv
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import misc

############ Global variables, into YAML file in the future ###############
# Visialusation
GLOBAL_MAP = True
LOCAL_MAP = False
WALL_WIDTH = 3



class ROS_MAPPING:

    sonar_data = 0
    ekf_data = 0

    def createImageGlobalMap(self):
        # Lagre array containing the global map
        globalMapDisplay = np.ones((5000,5000),np.float32)
        positionManta = [self.ekf_data.pose.y,self.ekf_data.pose.x]








    def createImageCurrentScan(self):
        # Array containing current scan data
        sonarDisplayArray = np.ones((1500,1000), np.float32)
        positionManta = [1498,499]
        angle_increment = self.sonar_data.angle_increment
        current_angle = self.sonar_data.angle_min
        while current_angle <= self.sonar_data.angle_min:
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

                # Each scan containts X-amount of pings with an increment in angle between the pings which have to be updated
                current_angle+=angle_increment 
        
            #Visualising sonar image
            cv.imshow("sonarDisplay",sonarDisplayArray)
            cv.waitKey(1)
            
    def sonarCallback(self,data):
        self.sonar_data = data
  
    def ekfCallback(self,data):
        self.ekf_data = data

    def __init__(self):
        rospy.Subscriber('manta/sonar',LaserScan,self.sonarCallback)
        rospy.Subscriber('/manta/pose_gt',Odometry,self.ekfCallback)
        if GLOBAL_MAP == True:
            self.createImageGlobalMap()
        elif LOCAL_MAP == True:
            self.createImageCurrentScan()
    
    




def mapping():
    rospy.init_node('sonar_mapping',anonymous=True)
    ROS_MAPPING()
    rospy.spin()

if __name__ == '__main__':
    mapping()



