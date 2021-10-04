#!/usr/bin/env python
# BEGIN ALL
from geometry_msgs.msg import PoseStamped
import numpy as np
import rospy
import math

class Motion_Estimation:
    def __init__(self):
        self.vicon_sub = rospy.Subscriber('/vrpn_client_node/racecar/pose',PoseStamped,self.Pos_callback)
        self.buffer_length = 6
        self.HZ = 120/3
        self.px_list = []
        self.py_list = []
        self.pz_list = []
        self.ox_list = []
        self.oy_list = []
        self.oz_list = []
        self.ow_list = []
        self.vx_list = []
        self.vy_list = []
        self.vz_list = []
        self.pitch_list = []
        self.roll_list = []
        self.yaw_list = []
        self.anvx_list = []
        self.anvy_list = []
        self.anvz_list = []

    def quaternion_rotation_matrix(self,ox,oy,oz,ow):
        """
        Covert a quaternion into a full three-dimensional rotation matrix.
 
        Input
        :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
        Output
        :return: A 3x3 element matrix representing the full 3D rotation matrix. 
                This rotation matrix converts a point in the local reference 
                frame to a point in the global reference frame.
        """
        # Extract the values from Q
        q0 = ox
        q1 = oy
        q2 = oz
        q3 = ow
     
        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)
     
        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)
     
        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
        # 3x3 rotation matrix
        rot_matrix = np.array([[r00, r01, r02],
                            [r10, r11, r12],
                            [r20, r21, r22]])
                            
        return rot_matrix
    
    def to_euler_angles(self, w, x, y, z):
        
        angles = {'pitch': 0.0, 'roll': 0.0, 'yaw': 0.0}
        r = math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
        p = math.asin(2*(w*y-x*z))
        y = math.atan2(2*(w*z+x*y),1-2*(z*z+y*y))

        angles['roll'] = r*180/math.pi
        angles['pitch'] = p*180/math.pi
        angles['yaw'] = y*180/math.pi
        #print(angles)
        return angles 

    def diff_pos(self):
        #linear
        ox_ave = np.mean(self.ox_list)
        oy_ave = np.mean(self.oy_list)
        oz_ave = np.mean(self.oz_list)
        ow_ave = np.mean(self.ow_list)
        rot_mat = self.quaternion_rotation_matrix(ox_ave,oy_ave,oz_ave,ow_ave)
        px_front = self.px_list[0:self.buffer_length/2]
        px_rear = self.px_list[self.buffer_length/2:self.buffer_length]
        py_front = self.py_list[0:self.buffer_length/2]
        py_rear = self.py_list[self.buffer_length/2:self.buffer_length]
        pz_front = self.py_list[0:self.buffer_length/2]
        pz_rear = self.py_list[self.buffer_length/2:self.buffer_length]
        px_diff = (np.sum(px_rear)-np.sum(px_front))/self.buffer_length*2
        py_diff = (np.sum(py_rear)-np.sum(py_front))/self.buffer_length*2
        pz_diff = (np.sum(pz_rear)-np.sum(pz_front))/self.buffer_length*2
        P_diff = np.array([px_diff,py_diff,pz_diff])
        P_diff = P_diff.reshape((3,1))
        rot_mat_inv = np.linalg.inv(rot_mat)
        linear_Velocity = np.dot(rot_mat_inv,P_diff)*self.HZ
        #print(rot_mat_inv.shape)
        #print(P_diff.shape)
    
        #angular
        roll_front = self.roll_list[0:self.buffer_length/2]
        roll_rear = self.roll_list[self.buffer_length/2:self.buffer_length]
        pitch_front = self.pitch_list[0:self.buffer_length/2]
        pitch_rear = self.pitch_list[self.buffer_length/2:self.buffer_length]
        yaw_front = self.yaw_list[0:self.buffer_length/2]
        yaw_rear = self.yaw_list[self.buffer_length/2:self.buffer_length]
        roll_diff = (np.sum(roll_rear)-np.sum(roll_front))/self.buffer_length*2
        pitch_diff = (np.sum(pitch_rear)-np.sum(pitch_front))/self.buffer_length*2
        yaw_diff = (np.sum(yaw_rear)-np.sum(yaw_front))/self.buffer_length*2
        roll_velo = roll_diff*self.HZ
        pitch_velo = pitch_diff*self.HZ
        yaw_velo = yaw_diff*self.HZ
        Velocity = np.array([linear_Velocity[0],linear_Velocity[1],linear_Velocity[2],roll_velo,pitch_velo,yaw_velo])
        self.Velo_callback(Velocity)
        #print(Velocity)
        #print(linear_Velocity)
        return Velocity
    
    def diff_velo(self):
        #linear
        vx_front = self.vx_list[0:self.buffer_length/2]
        vx_rear = self.vx_list[self.buffer_length/2:self.buffer_length]
        vy_front = self.vy_list[0:self.buffer_length/2]
        vy_rear = self.vy_list[self.buffer_length/2:self.buffer_length]
        vz_front = self.vz_list[0:self.buffer_length/2]
        vz_rear = self.vz_list[self.buffer_length/2:self.buffer_length]
        vx_diff = (np.sum(vx_rear)-np.sum(vx_front))/self.buffer_length*2
        vy_diff = (np.sum(vy_rear)-np.sum(vy_front))/self.buffer_length*2
        vz_diff = (np.sum(vz_rear)-np.sum(vz_front))/self.buffer_length*2
        ax = vx_diff*self.HZ
        ay = vy_diff*self.HZ
        az = vz_diff*self.HZ
        #angular
        anvx_front = self.anvx_list[0:self.buffer_length/2]
        anvx_rear = self.anvx_list[self.buffer_length/2:self.buffer_length]
        anvy_front = self.anvy_list[0:self.buffer_length/2]
        anvy_rear = self.anvy_list[self.buffer_length/2:self.buffer_length]
        anvz_front = self.anvz_list[0:self.buffer_length/2]
        anvz_rear = self.anvz_list[self.buffer_length/2:self.buffer_length]
        anvx_diff = (np.sum(anvx_rear)-np.sum(anvx_front))/self.buffer_length*2
        anvy_diff = (np.sum(anvy_rear)-np.sum(anvy_front))/self.buffer_length*2
        anvz_diff = (np.sum(anvz_rear)-np.sum(anvz_front))/self.buffer_length*2
        anax = anvx_diff*self.HZ
        anay = anvy_diff*self.HZ
        anaz = anvz_diff*self.HZ

        Accelaration = np.array([ax,ay,az,anax,anay,anaz])
        print(Accelaration)
        return Accelaration
    
    def Velo_callback(self,speed):
        vx = speed[0]
        vy = speed[1]
        vz = speed[2]
        anvx = speed[3]
        anvy = speed[4]
        anvz =speed[5]

        self.vx_list.append(vx)
        self.vy_list.append(vy)
        self.vz_list.append(vz)
        self.anvx_list.append(anvx)
        self.anvy_list.append(anvy)
        self.anvz_list.append(anvz)

        full = np.zeros(6)

        if(len(self.vx_list)>self.buffer_length):
            self.vx_list.pop(0)
            full[0] = 1
        if(len(self.vy_list)>self.buffer_length):
            self.vy_list.pop(0)
            full[1] = 1
        if(len(self.vz_list)>self.buffer_length):
            self.vz_list.pop(0)
            full[2] = 1
        if(len(self.anvx_list)>self.buffer_length):
            self.anvx_list.pop(0)
            full[3] = 1
        if(len(self.anvy_list)>self.buffer_length):
            self.anvy_list.pop(0)
            full[4] = 1
        if(len(self.anvz_list)>self.buffer_length):
            self.anvz_list.pop(0)
            full[5] = 1
        
        if(np.sum(full)==6):
            self.diff_velo()


    def Pos_callback(self,msg):
        px = msg.pose.position.x
        py = msg.pose.position.y
        pz = msg.pose.position.z
        ox = msg.pose.orientation.x
        oy = msg.pose.orientation.y
        oz = msg.pose.orientation.z
        ow = msg.pose.orientation.w
        angles = self.to_euler_angles(ow,ox,oy,oz)
        roll = angles['roll']
        pitch = angles['pitch']
        yaw = angles['yaw']
        
        self.px_list.append(px)
        self.py_list.append(py)
        self.pz_list.append(pz)
        self.ox_list.append(ox)
        self.oy_list.append(oy)
        self.oz_list.append(oz)
        self.ow_list.append(ow)
        self.roll_list.append(roll)
        self.pitch_list.append(pitch)
        self.yaw_list.append(yaw)

        full = np.zeros(10)

        if(len(self.px_list)>self.buffer_length):
            self.px_list.pop(0)
            full[0] = 1
        if(len(self.py_list)>self.buffer_length):
            self.py_list.pop(0)
            full[1] = 1
        if(len(self.pz_list)>self.buffer_length):
            self.pz_list.pop(0)
            full[2] = 1
        if(len(self.ox_list)>self.buffer_length):
            self.ox_list.pop(0)
            full[3] = 1
        if(len(self.oy_list)>self.buffer_length):
            self.oy_list.pop(0)
            full[4] = 1
        if(len(self.oz_list)>self.buffer_length):
            self.oz_list.pop(0)
            full[5] = 1
        if(len(self.ow_list)>self.buffer_length):
            self.ow_list.pop(0)
            full[6] = 1
        if(len(self.roll_list)>self.buffer_length):
            self.roll_list.pop(0)
            full[7] = 1
        if(len(self.pitch_list)>self.buffer_length):
            self.pitch_list.pop(0)
            full[8] = 1
        if(len(self.yaw_list)>self.buffer_length):
            self.yaw_list.pop(0)
            full[9] = 1
        
        if(np.sum(full)==10):
            self.diff_pos()


rospy.init_node('Motion_Estimation')
ME = Motion_Estimation()
rospy.spin()