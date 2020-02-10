#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
import calibration_misc
import mapping_functions
import cv2


class MANUAL_CALIBRATION:

    def calibrationCallback(self,data):
        # Visualising both sonar and camera feed
        image = calibration_misc.convert_to_openCV(self.image_data)
        mapping_functions.imageCurrentScan(self.sonar_data,WALL_WIDTH=3,SCALE=100)
        # Saving point
        calibration_misc.save_point



    def imageCallback(self,data):
        self.image_data = data

    def sonarCallback(self,data):
        self.sonar_data = data

    def __init__(self):        
        # Necesarry variables
        self.sonar_data = LaserScan()
        self.image_data = Image()
        self.y = []
        self.x = []
        
        # Necesarry callback functions
        rospy.Subscriber('manta/sonar',LaserScan,self.sonarCallback)
        rospy.Subscriber('manta/manta/camerafront/camera_image',Image,self.imageCallback)
        rospy.Timer(rospy.Duration(1.0/10.0),self.calibrationCallback)



if __name__ == "__main__":
    rospy.init_node('camera_sonar_calibration',anonymous=True)
    MANUAL_CALIBRATION()
    rospy.spin()



