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

class Zone_sel(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])
        zone_idx=1

    def execute(self, userdata):
        global pose
        pose=poses_list.pop()  
        Takeshi.define_pose(pose)
        return 'outcome1'

class S0(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2', 'outcome3', 'outcome4','outcome5','outcome6'])
        
    def execute(self, userdata):
        rospy.sleep(0.1)
        observation=Laser_.eval_laser_data(Laser_.get_laser_data())

        
        if observation[2]==1:
            print('Next state s1')
            return 'outcome1'
        elif observation[3]==1:           
            if Laser_.eval_laser_data(Laser_.get_laser_data(),limit=0.2)[3]==1:
                    print("publishing vel_y front left")
                    Takeshi.velocity_publisher(vel_y=-0.15) 
            print('Next state s4')        
            return 'outcome2'
        elif observation[1]==1:
            if Laser_.eval_laser_data(Laser_.get_laser_data(),limit=0.2)[1]==1:
                    print("publishing vel_y front right")
                    Takeshi.velocity_publisher(vel_y=0.15)
            print('Next state s6')        
            return 'outcome3'
        elif observation[4]==1:
            if Laser_.eval_laser_data(Laser_.get_laser_data(),limit=0.2)[4]==1:
                    print("publishing vel_y left")
                    Takeshi.velocity_publisher(vel_y=-0.15)        
            print('Next state s8')        
            return 'outcome4'
        elif observation[0]==1:
            if Laser_.eval_laser_data(Laser_.get_laser_data(),limit=0.2)[0]==1:
                    print("publishing vel_y right")
                    Takeshi.velocity_publisher(vel_y=0.15)
            print('Next state s9')        
            return 'outcome5' 
        else:
            print('Next state s10')
            return 'outcome6'

class S1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2'])
        
    def execute(self,userdata):
        print("Front obstacle")
        if Laser_.eval_laser_data(Laser_.get_laser_data())[2]==1:
            Takeshi.velocity_publisher(vel_x=-0.15)           
            return 'outcome1'
        return 'outcome2'

class S2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2'])
    
    def execute(self, userdata):
        rospy.sleep(0.1)
        takeshi_x, takeshi_y, _=Takeshi.obtain_coords()
        negative_direction= True if math.atan2((takeshi_y),(takeshi_x)) <0 else False
        custom_range=(600,630) if negative_direction else (90,120)
        eval_=Laser_.eval_laser_data(Laser_.get_laser_data(custom_range=custom_range))[5]==0
        
        if eval_:
            if negative_direction:
                Takeshi.velocity_publisher(w_z=-np.pi*0.25)
                return 'outcome1'
            else:
                Takeshi.velocity_publisher(w_z=np.pi*0.25)
                return 'outcome1'
        return 'outcome2'

class S3(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])
        
    def execute(self, userdata):
        rospy.sleep(0.1)    
        takeshi_x, takeshi_y, _=Takeshi.obtain_coords()
        negative_direction= True if math.atan2((takeshi_y),(takeshi_x)) <0 else False
        custom_range=(600,630) if negative_direction else (90,120)
        
        
        if Laser_.eval_laser_data(Laser_.get_laser_data(custom_range=custom_range), limit=1)[5]==1:
            Takeshi.velocity_publisher(vel_x=0.15)
        return 'outcome1'
    
class S4(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2'])
    
    def execute(self, userdata):
        print("Front left detection")
        rospy.sleep(0.1)    
        if Laser_.eval_laser_data(Laser_.get_laser_data(custom_range=(600,630)))[5]==0:
            Takeshi.velocity_publisher(w_z=-np.pi*0.25)
            return 'outcome1'
        return 'outcome2'
        
class S5(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])
    
    def execute(self, userdata):
        rospy.sleep(0.1)    
        if Laser_.eval_laser_data(Laser_.get_laser_data(custom_range=(600,630)), limit=1)[5]==1:
            Takeshi.velocity_publisher(vel_x=0.15)
        return 'outcome1'    
        
class S6(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2'])
    
    def execute(self, userdata):
        print("Front right detection")
        rospy.sleep(0.1)    
        if Laser_.eval_laser_data(Laser_.get_laser_data(custom_range=(90,120)))[5]==0:
            Takeshi.velocity_publisher(w_z=np.pi*0.25)      
            return 'outcome1'
        return 'outcome2'
        
class S7(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])
    
    def execute(self, userdata):
        rospy.sleep(0.1)    
        if Laser_.eval_laser_data(Laser_.get_laser_data(custom_range=(90,120)), limit=0.8)[5]==1:
            Takeshi.velocity_publisher(vel_x=0.15)
        return 'outcome1'          
        
class S8(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])
    
    def execute(self, userdata):
        print("Left detection")
        rospy.sleep(0.1)    
        if Laser_.eval_laser_data(Laser_.get_laser_data(custom_range=(600,630)), limit=0.8)[5]==1:
            Takeshi.velocity_publisher(vel_x=0.15)
        return 'outcome1'
        
class S9(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])
    
    def execute(self, userdata):
        print("Right detection")
        rospy.sleep(0.1)    
        if Laser_.eval_laser_data(Laser_.get_laser_data(custom_range=(90,120)), limit=0.8)[5]==1:
            Takeshi.velocity_publisher(vel_x=0.15)
        return 'outcome1'       
    
class S10(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2'])
        
    def execute(self, userdata):
        x=pose.pose.position.x
        y=pose.pose.position.y
        z=pose.pose.orientation.z
        takeshi_x, takeshi_y, takeshi_phi=Takeshi.obtain_coords()
        print(f'Ubicación meta x:{round(x,2)}, y:{round(y,2)}, θz:{round(z,2)})')        
        print(f"Distancia restante: {math.sqrt((takeshi_x)**2 + (takeshi_y)**2)}")
        Takeshi.velocities_from_coords(takeshi_x,takeshi_y,takeshi_phi,x,y)
        print()
        print(f'Ubicación meta ({x} {y})')
        print(f'Ubicación actual ({round(takeshi_x,4)}, {round(takeshi_y,4)})')
        print(f'Diferencia X: {round(takeshi_x,4)}, Diferencia Y:{round(takeshi_y,4)}')


        not_reached=True
#        if abs(takeshi_x) <0.1:
#            if abs(takeshi_y)<0.1:
#                not_reached=False
#
        if math.sqrt((takeshi_x)**2 + (takeshi_y)**2) < 1.4:
            not_reached=False
               
        if not_reached:
            print("Moviendo")    
            Takeshi.velocities_from_coords(takeshi_x,takeshi_y, takeshi_phi,x,y)
            takeshi_x, takeshi_y, takeshi_phi=Takeshi.obtain_coords()
            return 'outcome1'
        else:      
            return 'outcome2'
        rospy.spin()

class S11(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2'])
              
    def execute(self, userdata):
        print("Alineando")
#        _, _, takeshi_phi=Takeshi.obtain_coords()
#        wz=0.2*np.pi if takeshi_phi > 0 else -0.2*np.pi
        takeshi_x, takeshi_y, takeshi_phi=Takeshi.obtain_coords()
        phi=math.atan2((takeshi_y),(takeshi_x))
        print(f'Ángulo restante para alinear es: {phi}')
        wz=0.9*(phi) 
        
        if abs(phi)>0.1:
            print(f"Moviendo en z, ángulo restante:{takeshi_phi}")
            Takeshi.velocity_publisher(w_z=wz)
            print("Meta alcanzada")          
            return 'outcome1'
        else:
            return 'outcome2'

class Frame_returner(smach.State):
     def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2'])
        self.zone_index=1
        
     def execute(self, userdata):
        Takeshi.velocity_publisher(0,0,0)
        print(f"Moving head to {pose.pose.position.z}")
        Takeshi.move_head(pose.pose.position.x,pose.pose.position.y,pose.pose.position.z)
        if pose.pose.position.z > 0.3:
            rgbd.get_mask_with_depth()
        else:
            rgbd.get_mask_no_depth()
        
        cent_list=rgbd.get_centroid()
        text=f'Zona {self.zone_index}'
        txt_dict[text]=[cent_list]
        self.zone_index+=1
        Takeshi.head_restore()

        if poses_list:
            return 'outcome1'
        else:
            txt.create_txt(txt_dict)
            return 'outcome2'      


class END(smach.State):
    def __init__(self):
        smach.State.__init__(self)
              
    def execute(self, userdata):
        return
        

def define_objects(Takeshi_ob, Laser_ob, poses,Rgbd_ob, txt_ob):
    global Takeshi, Laser_, poses_list,rgbd, txt, txt_dict, zone_index
    txt_dict={}
    poses_list=poses
    Takeshi=Takeshi_ob
    Laser_=Laser_ob
    rgbd=Rgbd_ob
    txt=txt_ob
    
        
       
