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



import cv2
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

import moveit_commander
import moveit_msgs.msg






class TXT:
    def __init__(self):
        self.lines=['Bienvenidos'.center(40), 'Equipo: Metzi BUAP'.center(40), 'Frames generados respecto al mapa']
        self.actual_zone=None
        self.object_idx=None        
    
    def create_txt(self, txt_dict):
        file_path = 'src/metzi_buap_etapa03/scripts/Frames.txt'
        print("Creating TXT")
        if os.path.isfile(file_path):
            os.remove(file_path)
            print("File has been deleted")
        else:
            print("File does not exist")
        with open(file_path, 'a') as f:
            
            for key in txt_dict.keys():
                self.lines.append(key)
                for coords in txt_dict[key]:
                    try:
                        self.lines.append(f'\tObjeto {1} ==> x:{coords[0][0]},y:{coords[0][1]},z:{coords[0][2]}')
                        self.lines.append(f'\tObjeto {2} ==> x:{coords[1][0]},y:{coords[1][1]},z:{coords[1][2]}')
                    except:
                        pass
            f.write('\n'.join(self.lines))
        f.close()

class Laser():
    def __init__(self):
        print("Laser class created")        
    
    def get_data(self):
        return rospy.wait_for_message('/hsrb/base_scan', LaserScan, rospy.Duration(2.0))
    
    def get_laser_data(self, custom_range=(629,630)):
        lecture=np.asarray(self.get_data().ranges)
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
        ## definir las zonas de la medición
        ## Hacer observación basada en la trasnformación y cada uno de los puntos
        ## asignar 1 si punto está en cierta distancia 
        ## 
        for i in range(len(laser_data)):
            if np.mean(laser_data[i]) < limit:
                observation[i]=1
        return observation
        
    
class RGBD():

    def __init__(self, listener, broadcaster, tfbuffer2, listener2):
        self._cloud_sub = rospy.Subscriber(
            "/hsrb/head_rgbd_sensor/depth_registered/rectified_points",
            PointCloud2, self._cloud_cb)
        self.listener=listener
        self.broadcaster=broadcaster
        self.listener2=listener2
        self.tfbuffer2=tfbuffer2
        self._points_data = None
        self._image_data = None
        self._h_image = None
        self._region = None
        self._h_min = 0
        self._h_max = 0
        self._hsv_iamge=None
        self._xyz = [0, 0, 0]

        self.ob_idx=0
        self.final_mask=None
        self.cent=None
        self.txt_dict={}

    def _cloud_cb(self, msg):
        self._points_data = ros_numpy.numpify(msg)

        self._image_data = \
            self._points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]

        self._hsv_image = cv2.cvtColor(self._image_data, cv2.COLOR_RGB2HSV_FULL)
        self._h_image = self._hsv_image[..., 0]

    def get_image(self):
        return self._image_data
    
    def get_region(self, h_min, h_max):
        self._region = \
            (self._h_image > h_min) & (self._h_image < h_max)
        return self._region
    
    def get_mask_with_depth(self):
        depth_values=self._points_data['z']
        cm_hsv=self.most_common_hsv()
        
        region_h = (self._h_image != cm_hsv[0])
        region_d = depth_values < 2.5
        
        idx, idy=np.where(np.logical_and(region_d, region_h))

        mask=np.zeros((480,640))
        mask[idx,idy]=255
        kernel = np.ones((6, 6), np.uint8)
        eroded_mask=cv2.erode(mask,kernel)
        dilated_mask=cv2.dilate(eroded_mask,kernel)        
        self.final_mask=dilated_mask

        cv2.imshow("Masked image", self.final_mask)
        cv2.waitKey(5000)
        cv2.destroyAllWindows()    
    
    def get_mask_no_depth(self):
        cm_hsv=self.most_common_hsv()
        region_h = (self._h_image != cm_hsv[0])
        idx, idy=np.where((region_h))
        mask=np.zeros((480,640))
        mask[idx,idy]=255
        kernel_dil = np.ones((10, 10), np.uint8)
        kernel_erode = np.ones((6, 6), np.uint8)
        eroded_mask=cv2.erode(mask,kernel_erode)
        dilated_mask=cv2.dilate(eroded_mask,kernel_dil)
        self.final_mask=dilated_mask

        cv2.imshow("Masked image", self.final_mask)


    def get_points(self):
        return self._points_data

    def get_h_image(self):
        return self._h_image
    
    def get_hsv_image(self):
        return self._hsv_image

    def get_xyz(self):
        return self._xyz

    
    def get_centroid(self):
        contours, hierarchy = cv2.findContours(self.final_mask.astype('uint8'),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        print(f'Number of contours: {len(contours)}')
        
        points=self._points_data
        img=self.get_hsv_image()
        coord_list=[]
        
  
        for contour in contours:
            xz=[]
            M = cv2.moments(contour)  
            
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            
            cv2.circle(img, (cX, cY), 5, (255, 255, 255), -1)
                       
            x=cX
            y=cY
            w=h=5
            
            cv2.rectangle(img, (x, y), (x + w, y + h), (0,0,255), 2)
            cv2.imshow("bound rect image", img)
            cv2.waitKey(3000)
            cv2.destroyAllWindows()

            for jy in range (x, x+w):
               for ix in range(y, y+h):
                    aux=(np.asarray((points['x'][ix,jy], points['y'][ix,jy], points['z'][ix,jy])))
                    if np.isnan(aux[0]) or np.isnan(aux[1]) or np.isnan(aux[2]):
                        'reject point'
                    else:
                        xz.append(aux)
                        
            xz=np.asarray(xz)
            cent_xz=xz.mean(axis=0)
            cent=cent_xz
            coord = self.publish_frame(cent)
            coord_list.append(coord)
            
        return coord_list
             
    def publish_frame(self, cent):
        x,y,z=cent
        
        if np.isnan(x) or np.isnan(y) or np.isnan(z):
            print('nan')
        else:
            name= 'obj'+str(self.ob_idx)
            fix_name= 'obj_fixed'+str(self.ob_idx)
            self.broadcaster.sendTransform((x,y,z),(0,0,0,1), rospy.Time.now(), name,"head_rgbd_sensor_link")
            self.ob_idx+=1
            time.sleep(1)
            t,_=self.listener.lookupTransform('map',name, rospy.Time(0))
            print(t)
            self.broadcaster.sendTransform((t[0], t[1], t[2]),(0,0,0,1), rospy.Time(0), fix_name,'map')
            return (t[0], t[1], t[2])        

    def most_common_hsv(self):
        hsv_temp = self._hsv_image.copy()
        unique, counts = np.unique(hsv_temp.reshape(-1, 3), axis=0, return_counts=True)
        self.most_commons_h=unique[np.argmax(counts)]
        return self.most_commons_h 

    def show_img_compar(self,img_1, img_2 ):
        f, ax = plt.subplots(1, 2, figsize=(10,10))
        ax[0].imshow(img_1)
        ax[1].imshow(img_2)
        ax[0].axis('off')
        ax[1].axis('off')
        f.tight_layout()       
        
        
class Takeshi_utils:
    def __init__(self, tf_buffer2, listener2,listener, base_vel_publisher, head):
        print("Takeshi created")
        self.twist_msg=Twist()
        self.tf_buffer2=tf_buffer2
        self.listener2=listener2
        self.listener=listener
        self.pose=None
        self.base_vel_publisher=base_vel_publisher
        self.head=head

    def velocities_from_coords(self, takeshi_x, takeshi_y, takeshi_phi, x, y):
        phi=math.atan2((takeshi_y),(takeshi_x))
        print(f'Ángulo es {phi}')
        w_z=2.5*(phi)
        vel_x=0.2*math.sqrt((takeshi_x)**2 + (takeshi_y)**2)
        self.velocity_publisher(vel_x=vel_x, vel_y=0, w_z=w_z)

    def velocity_publisher(self, vel_x=0, vel_y=0, w_z=0):
        self.twist_msg.linear.x=vel_x
        self.twist_msg.linear.y=vel_y
        self.twist_msg.angular.z=w_z
        print(f"Velocidad en x:{round(self.twist_msg.linear.x,4)}, y:{round(self.twist_msg.linear.y,4)}, velocidad angular:{round(self.twist_msg.angular.z,4)}")
        self.base_vel_publisher.publish(self.twist_msg)
        
    def obtain_coords(self):
        for i in range(10):
            baselink_trans = self.tf_buffer2.lookup_transform('base_link', self.pose.header.frame_id,self.pose.header.stamp,rospy.Duration(4))
            takeshi_pose=tf2_geometry_msgs.do_transform_pose(self.pose,baselink_trans)
            takeshi_x=takeshi_pose.pose.position.x
            takeshi_y=takeshi_pose.pose.position.y
            takeshi_phi=tf.transformations.euler_from_quaternion([0, 0, takeshi_pose.pose.orientation.z, takeshi_pose.pose.orientation.w])[2]
        return takeshi_x,takeshi_y, takeshi_phi
    
    def define_pose(self,pose):
        self.pose=pose
        
    def head_restore(self):
        head_pose = self.head.get_current_joint_values()
        head_pose[0]=0.0
        head_pose[1]=0.0
        self.head.set_joint_value_target(head_pose)
        self.head.go()        
        
    def move_head(self,x,y,z):
        self.head_restore()
        head_pose = self.head.get_current_joint_values()
        
        trans, rot = self.listener.lookupTransform('/map', '/head_rgbd_sensor_gazebo_frame', rospy.Time(0))
        
        e =tf.transformations.euler_from_quaternion(rot)    

        x_rob,y_rob,z_rob,th_rob= trans[0], trans[1] ,trans[2] ,  e[2]
        
        D_x=x_rob-x
        D_y=y_rob-y
        D_z=z_rob-z

        D_th= np.arctan2(D_y,D_x)

        pan_correct= (- th_rob + D_th + np.pi) % (2*np.pi)

        if(pan_correct > np.pi):
            pan_correct=-2*np.pi+pan_correct
        if(pan_correct < -np.pi):
            pan_correct=2*np.pi+pan_correct

        if ((pan_correct) > .5 * np.pi):
            print ('Exorcist alert')
            pan_correct=.5*np.pi
        head_pose[0]=pan_correct
        tilt_correct=np.arctan2(D_z,np.linalg.norm((D_x,D_y)))

        head_pose [1]=-tilt_correct    
        
        self.head.set_joint_value_target(head_pose)
        succ=self.head.go()
        return succ
        
        
        
        
        
        
        
        
