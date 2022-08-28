#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
import tf

import math
import numpy as np
#import ros_numpy

#import cv2
#from cv_bridge import CvBridge

import time
import os

from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
#from sensor_msgs.msg import PointCloud2
#from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

import smach

from utils import *

class S0(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2', 'outcome3', 'outcome4','outcome5','outcome6'])
        
    def execute(self, userdata):
        rospy.sleep(0.1)
        observation=Laser_.eval_laser_data(Laser_.get_laser_data())
        print(observation)
        
        if observation[2]==1:
            print('Next state s1')
            return 'outcome1'
        elif observation[3]==1:
            print('Next state s4')        
            return 'outcome2'
        elif observation[1]==1:
            print('Next state s6')        
            return 'outcome3'
        elif observation[4]==1:
            print('Next state s8')        
            return 'outcome4'
        elif observation[0]==1:
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
            Takeshi.velocity_publisher(vel_x=-0.1)           
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
        print('negativo' if negative_direction==True else 'positivo')
        eval_=Laser_.eval_laser_data(Laser_.get_laser_data(custom_range=custom_range))[5]==0
        print(eval_)
        
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
        
        if Laser_.eval_laser_data(Laser_.get_laser_data(custom_range=custom_range), limit=0.8)[5]==1:
            Takeshi.velocity_publisher(vel_x=0.1)
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
        if Laser_.eval_laser_data(Laser_.get_laser_data(custom_range=(600,630)), limit=0.8)[5]==1:
            Takeshi.velocity_publisher(vel_x=0.1)
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
            Takeshi.velocity_publisher(vel_x=0.1)
        return 'outcome1'          
        
class S8(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])
    
    def execute(self, userdata):
        print("Left detection")
        rospy.sleep(0.1)    
        if Laser_.eval_laser_data(Laser_.get_laser_data(custom_range=(600,630)), limit=0.8)[5]==1:
            Takeshi.velocity_publisher(vel_x=0.1)
        return 'outcome1'
        
class S9(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])
    
    def execute(self, userdata):
        print("Right detection")
        rospy.sleep(0.1)    
        if Laser_.eval_laser_data(Laser_.get_laser_data(custom_range=(90,120)), limit=0.8)[5]==1:
            Takeshi.velocity_publisher(vel_x=0.1)
        return 'outcome1'       
    
class S10(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2'])
        
    def execute(self, userdata):
        x=goal.pose.position.x
        y=goal.pose.position.y
        z=goal.pose.orientation.z
        takeshi_x, takeshi_y, takeshi_phi=Takeshi.obtain_coords()
        print(f'Ubicación meta x:{round(x,2)}, y:{round(y,2)}, θz:{round(z,2)})')        
        print(f"Distancia restante: {math.sqrt((takeshi_x)**2 + (takeshi_y)**2)}")
        Takeshi.velocities_from_coords(takeshi_x,takeshi_y,takeshi_phi,x,y)
        print()
        print(f'Ubicación meta ({x} {y}')
        print(f'Ubicación actual ({takeshi_x}, {takeshi_y})')
        print(f'Diferencia X: {takeshi_x}, Diferencia Y:{takeshi_y}')
        print(takeshi_x)
        print(takeshi_y)
        not_reached=True
        if abs(takeshi_x) <0.1:
            if abs(takeshi_y)<0.1:
                not_reached=False
        print(not_reached)
       
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
        _, _, takeshi_phi=Takeshi.obtain_coords()
        wz=0.2*np.pi if takeshi_phi > 0 else -0.2*np.pi
        if abs(takeshi_phi)>0.15:
            print(f"Moviendo en z, ángulo restante:{takeshi_phi}")
            Takeshi.velocity_publisher(w_z=wz)
            print("Meta alcanzada")          
            return 'outcome1'
        else:
            return 'outcome2'
            
class END(smach.State):
    def __init__(self):
        smach.State.__init__(self)
              
    def execute(self, userdata):
        return 
        

class Laser_data:
    def get_laser_data(self, custom_range=(629,630)):
        lecture=np.asarray(laser.get_data().ranges)
        lecture=np.where(lecture>4.5,4.5,lecture)
        right=lecture[90:198]
        front_right=lecture[198:306]
        front=lecture[306:414]
        front_left=lecture[414:522]
        left=lecture[522:630]
        custom_lecture=lecture[custom_range[0]:custom_range[1]]
        return(right,front_right,front,front_left,left, custom_lecture)
     
    def eval_laser_data(self, laser_data, limit=0.5):
        observation=np.zeros(shape=(6), dtype=int)
        for i in range(len(laser_data)):
            if np.mean(laser_data[i]) < limit:
                observation[i]=1
        print(observation)
        return observation
        
class Takeshi_utils:
    def __init__(self):
        print("Takeshi created")
        self.twist_msg=Twist()
        #self.laser=Laser()

    def velocities_from_coords(self, takeshi_x, takeshi_y, takeshi_phi, x, y):
        phi=math.atan2((takeshi_y),(takeshi_x))
        print(f'Ángulo es {phi}')
        w_z=2.5*(phi)
        vel_x=0.2*math.sqrt((takeshi_x)**2 + (takeshi_y)**2)
        self.velocity_publisher(vel_x, w_z)

    def velocity_publisher(self, vel_x=0, w_z=0):
        self.twist_msg.linear.x=vel_x
        self.twist_msg.angular.z=w_z
        print(f"Velocidad en x:{self.twist_msg.linear.x}, velocidad angular:{self.twist_msg.angular.z}")
        base_vel_publisher.publish(self.twist_msg)
        
    def obtain_coords(self):
        for i in range(10):
            baselink_trans = tf_buffer.lookup_transform('base_link', goal.header.frame_id,goal.header.stamp,rospy.Duration(4.0))
            takeshi_pose=tf2_geometry_msgs.do_transform_pose(goal,baselink_trans)
            takeshi_x=takeshi_pose.pose.position.x
            takeshi_y=takeshi_pose.pose.position.y
            takeshi_phi=tf.transformations.euler_from_quaternion([0, 0, takeshi_pose.pose.orientation.z, takeshi_pose.pose.orientation.w])[2]
        return takeshi_x,takeshi_y, takeshi_phi        

   
def main():
    print("Initialazed")
    #os.system('clear')
    global goal, base_vel_publisher, Takeshi, Laser_
    goal = rospy.wait_for_message('/meta_competencia', PoseStamped, timeout=None)
    
    transform=tf_buffer.lookup_transform('base_link', goal.header.frame_id,goal.header.stamp,rospy.Duration(1.5))
    goal_transformed=tf2_geometry_msgs.do_transform_pose(goal,transform)
    
    base_vel_publisher = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)
    Laser_=Laser_data()

    Takeshi=Takeshi_utils()
    


        
if __name__ == '__main__':
    os.system('clear')
    rospy.init_node('metzi_eval1')
    laser=Laser()
    sm=smach.StateMachine(outcomes=['END'])   
    
    tf_buffer=tf2_ros.Buffer(rospy.Duration(1.5))
    listener = tf2_ros.TransformListener(tf_buffer)
    

    
    main()
    
    with sm:
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
        smach.StateMachine.add('s11', S11(), transitions={'outcome1':'s0', 'outcome2':'END'})
    
outcome=sm.execute()
    
       
    
    
