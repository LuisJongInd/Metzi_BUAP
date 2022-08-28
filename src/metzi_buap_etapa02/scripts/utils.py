#!/usr/bin/env python3

from sensor_msgs.msg import LaserScan
import rospy

class Laser():
    def __init__(self):
        print("Laser class created")        
    
    def get_data(self):
        return rospy.wait_for_message('/hsrb/base_scan', LaserScan, rospy.Duration(2.0))
