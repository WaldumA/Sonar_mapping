#!/usr/bin/env python

import rospy
import cv2 as cv
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import misc
import mapping_functions

############ Global variables, into YAML file in the future ###############
# Visialusation
GLOBAL_MAP = True
LOCAL_MAP = True
PUBLISH_GLOBAL_MAP = False
PUBLISH_LOCAL_MAP = False
WALL_WIDTH = 3
SCALE = 100
###########################################################################




class ROS_MAPPING:

    #### Class attribtues - Constant variables - Read from YAML #####



    #################################################################

    #### Class handeling functions ##########################

    def createImageGlobalMap(self):
        mapping_functions.imageGlobalMap(self.sonar_data,self.ekf_data,self.map,WALL_WIDTH,SCALE)

    def createImageCurrentScan(self):
        mapping_functions.imageCurrentScan(self.sonar_data,WALL_WIDTH,SCALE)

    def publishGlobalMap():
        pass

    def publishLocalMap():
        pass
        
    def sonarCallback(self,data):
        self.sonar_data = data
  
    def ekfCallback(self,data):
        self.ekf_data = data

    def mappingCallback(self,data): 
        if GLOBAL_MAP == True:
            self.createImageGlobalMap()
        if LOCAL_MAP == True:
            self.createImageCurrentScan()   
        if PUBLISH_GLOBAL_MAP == True:
            self.publishGlobalMap()
        if PUBLISH_LOCAL_MAP == True:
            self.publishLocalMap()

    def __init__(self):
        self.sonar_data = LaserScan()
        self.ekf_data = Odometry()
        self.map = np.ones((5000,5000),np.float32)
        rospy.Subscriber('manta/sonar',LaserScan,self.sonarCallback)
        rospy.Subscriber('odometry/filtered',Odometry,self.ekfCallback)
        rospy.Timer(rospy.Duration(1.0/10.0),self.mappingCallback)
        

#### INIT of node ##################################
if __name__ == '__main__':
    rospy.init_node('sonar_mapping',anonymous=True)
    ROS_MAPPING()
    rospy.spin()
###################################################

