#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
import tf

import math
import numpy as np
import ros_numpy
import matplotlib.pyplot as plt
from gazebo_ros import gazebo_interface



#import cv2
#from cv_bridge import CvBridge

import time
import os

from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2,LaserScan
#from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped,TransformStamped, Quaternion, Pose
    
import smach

from utils import *

import moveit_commander
import moveit_msgs.msg

from state_machine import *

def main():
    print("Initializing")
    #os.system('clear')
    global base_vel_publisher, Takeshi, Laser_
    
    base_vel_publisher = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)
    Laser_=Laser()

    Takeshi=Takeshi_utils(tf_buffer2, listener2,listener, base_vel_publisher, head)
    Rgbd_ob=RGBD(listener,broadcaster, tf_buffer2, listener2)
    txt_ob=TXT()
    define_objects(Takeshi, Laser_, poses, Rgbd_ob, txt_ob)
    

if __name__ == '__main__':
    os.system('clear')
    rospy.init_node('metzi_eval2')

    sm=smach.StateMachine(outcomes=['END']) 
    head = moveit_commander.MoveGroupCommander('head')
    arm = moveit_commander.MoveGroupCommander('arm')
    print("Setting arm to resting position")
    arm.set_named_target('go')
    arm.go()
    
    tf_buffer2=tf2_ros.Buffer(rospy.Duration(1.5))
    listener2=tf2_ros.TransformListener(tf_buffer2)
    
    listener = tf.TransformListener()
    broadcaster= tf.TransformBroadcaster()
    
    dict_poses={1:(0,1.21,0.5),2:(-3,4,0),3:(3.9,5.6,0)}
    poses=[]
    count=0
    for value in dict_poses.values():
        pose=PoseStamped()
        pose.header.frame_id='map'
        pose.pose.position.x=value[0]
        pose.pose.position.y=value[1]
        pose.pose.position.z=value[2]
        count+=1
        poses.insert(0, pose)
    
    main()
    
    with sm:
        smach.StateMachine.add('zone_sel', Zone_sel(),transitions={'outcome1':'s0'})
        smach.StateMachine.add('s0', S0(), transitions={'outcome1':'s1','outcome2':'s4','outcome3':'s6','outcome4':'s8','outcome5':'s9','outcome6':'s10'})
        smach.StateMachine.add('s1', S1(), transitions={'outcome1':'s0','outcome2':'s2'})
        smach.StateMachine.add('s2', S2(), transitions={'outcome1':'s0','outcome2':'s3'})
        smach.StateMachine.add('s3', S3(), transitions={'outcome1':'s0'})
        smach.StateMachine.add('s4', S4(), transitions={'outcome1':'s0','outcome2':'s5'})
        smach.StateMachine.add('s5', S5(), transitions={'outcome1':'s0'})
        smach.StateMachine.add('s6', S6(), transitions={'outcome1':'s0','outcome2':'s7'})
        smach.StateMachine.add('s7', S7(), transitions={'outcome1':'s0'})
        smach.StateMachine.add('s8', S8(), transitions={'outcome1':'s0'})
        smach.StateMachine.add('s9', S9(), transitions={'outcome1':'s0'})
        smach.StateMachine.add('s10', S10(), transitions={'outcome1':'s0','outcome2':'s11'})
        smach.StateMachine.add('s11', S11(), transitions={'outcome1':'s0','outcome2':'frame_returner'})
        smach.StateMachine.add('frame_returner', Frame_returner(), transitions={'outcome1':'zone_sel','outcome2':'END'})

    
outcome=sm.execute()



