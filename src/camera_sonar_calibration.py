#!/usr/bin/env python
import os
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
import calibration_misc
import mapping_functions
import cv2


class MANUAL_CALIBRATION:

    def calibrationCallback(self,data):
        # Get input from user
        user_input = -1
        user_input = input()
        output = calibration_misc.input_switcher(user_input)

        # A switch case for what to do with user input
        if output == 'Save':
            current_width, current_bearing, current_range = calibration_misc.Save_current_frame(self.sonar_data, self.image_data)
        elif output == 'Calibrate':
            print("Mello")











   
    def visualisingCallback(self,data):
        # Visualising both sonar and camera feed
        calibration_misc.convert_to_openCV(self.image_data)
        mapping_functions.imageCurrentScan(self.sonar_data,WALL_WIDTH=3,SCALE=100)

    def imageCallback(self,data):
        self.image_data = data

    def sonarCallback(self,data):
        self.sonar_data = data

    def __init__(self):        
        # Necesarry variables
        self.sonar_data = LaserScan()
        self.image_data = Image()
        self.image = 0
        self.pixel_width = []
        self.sonar_bearing = []
        self.sonar_range = []
        
        # Necesarry callback functions
        rospy.Subscriber('manta/sonar',LaserScan,self.sonarCallback)
        rospy.Subscriber('manta/manta/camerafront/camera_image',Image,self.imageCallback)
        #rospy.Timer(rospy.Duration(1.0/10.0),self.visualisingCallback)
        rospy.Timer(rospy.Duration(1.0/10.0),self.calibrationCallback)



if __name__ == "__main__":
    rospy.init_node('camera_sonar_calibration',anonymous=True)
    MANUAL_CALIBRATION()
    rospy.spin()



