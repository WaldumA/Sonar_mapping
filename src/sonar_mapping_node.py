#!/usr/bin/env python
import rospy
from std_msgs.msg import String
#from sensor_msgs import LaserScan
from sensor_msgs.msg import LaserScan
def sonarCallback(data):
    print(data.data)

def mapping():
    rospy.init_node('sonar_mapping',anonymous=True)
    rospy.Subscriber('manta/sonar',LaserScan,sonarCallback)
    rospy.spin()

if __name__ == '__main__':
    mapping()



