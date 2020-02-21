#!/usr/bin/env python
import rospy
from vortex_msgs.msg import ObjectPlacement
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
import object_placement_misc

################################################
# Globale variabler
WIDTH = 500
HEIGHT = 500
SCALE = 10
a = 9.19565983649e-10
b = -1.58986508509e-06
c = -0.000688394733634
d = 0.740485051656
################################################


class OBJECT_PLACING:
    def __init__(self):

        # Variables
        
        self.sonar_data = LaserScan()
        self.image = Image()
        self.ekf_data = Odometry()

        # Publshers
        self.pub = rospy.Publisher('/manta/object_position',ObjectPlacement,queue_size=10)

        # Subscribers
        rospy.Subscriber('manta/sonar',LaserScan,self.sonarCallback)
        rospy.Subscriber('/odometry/filtered',Odometry,self.ekfCallback)
        rospy.Subscriber('/manta/manta/camerafront/camera_image',Image,self.imageCallback)
        rospy.Timer(rospy.Duration(1.0/10.0),self.placeObjectCallback)

    # Checks if YOLOv3 is able to detect an object, AND returns the width pixel of the object. 
    def placeObjectCallback(self,data):
        try:
            [statement, pixel_width] = object_placement_misc.checkForObject(self.image) # Placeholder function will subscribe to topic in final code
        except:
            statement = False
            print("Can't find image stream")
        if statement:
            bearing = object_placement_misc.find_sonar_pos(pixel_width,a,b,c,d)
            print(bearing)
            depth = object_placement_misc.find_sonar_depth(self.sonar_data,bearing)
            if depth != False:
                obj_pos_msg = object_placement_misc.find_object_position(bearing,depth,self.ekf_data,WIDTH,HEIGHT,SCALE,self.sonar_data)
                self.pub.publish(obj_pos_msg)


    def imageCallback(self,data):
        self.image = data

    def sonarCallback(self,data):
        self.sonar_data = data
  
    def ekfCallback(self,data):
        self.ekf_data = data

    
if __name__ == '__main__':
    rospy.init_node('place_object_in_map',anonymous=True)
    OBJECT_PLACING()
    rospy.spin()














####### Demo Object While waiting for Object detection #################