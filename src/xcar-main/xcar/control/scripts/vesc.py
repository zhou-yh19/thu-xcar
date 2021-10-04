
# BEGIN ALL
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, Bool
from vesc_msgs.msg import VescCtrlStamped
from geometry_msgs.msg import PoseStamped
import queue
from scipy.spatial.transform import Rotation as R
import math
import numpy as np

class PIDdrift:
    def __init__(self):
        self.laser_sub = rospy.Subscriber('/scan',LaserScan,self.laser_callback)
        self.pose_sub = rospy.Subscriber('/vrpn_client_node/racecar/pose',PoseStamped,self.pose_callback, queue_size=1)
        self.drive_pub = rospy.Publisher('/vesc/ctrl', VescCtrlStamped, queue_size=1)
        self.ctrl = VescCtrlStamped()
        self.last_d = 0
        self.intg = 0
        self.safe = 0 # 0 for safe, 1 for unsafe
        self.horizon = 120
        self.psid_Queue = queue.Queue(self.horizon)
        self.vx_Queue = queue.Queue(self.horizon)
        self.vy_Queue = queue.Queue(self.horizon)
        self.last_psi = 0
        self.last_x = 0
        self.last_y = 0
        self.target_d = 1.5
    
    def laser_callback(self, msg):
        ranges = msg.ranges[270:810]
        distance = min(ranges)
        if(distance < 0.6):
            self.safe = 1
    
    def pose_callback(self, msg):
        x = msg.pose.position.x 
        y = msg.pose.position.y
        ori = msg.pose.orientation
        q = [ori.x, ori.y, ori.z, ori.w]
        r = R.from_quat(q)
        psi, _, _ = r.as_euler('zyx')
        if(self.psid_Queue.qsize() == 0):
            [self.psid_Queue.put(0.0) for i in range(self.horizon)]
            [self.vx_Queue.put(0.0) for i in range(self.horizon)]
            [self.vy_Queue.put(0.0) for i in range(self.horizon)]
        else:
            self.psid_Queue.get()
            if((psi - self.last_psi) < np.pi):
                real_psid = ((2 * np.pi) + psi - self.last_psi) * 120
            elif((psi - self.last_psi) > np.pi):
                real_psid = ((-2 * np.pi) + psi - self.last_psi) * 120
            else:
                real_psid = (psi - self.last_psi) * 120
            self.psid_Queue.put(real_psid)
            self.vx_Queue.get()
            self.vx_Queue.put((x - self.last_x) * 120)
            self.vy_Queue.get()
            self.vy_Queue.put((y - self.last_y) * 120)

        self.last_x = x
        self.last_y = y
        self.last_psi = psi

        psid_list = list(self.psid_Queue.queue)
        psid = sum(psid_list) / self.horizon
        vx_list = list(self.vx_Queue.queue)
        vx = sum(vx_list) / self.horizon
        vy_list = list(self.vy_Queue.queue)
        vy = sum(vy_list) / self.horizon
        '''
        v = math.sqrt(vx ** 2 + vy ** 2)
        print(v,' ',psid)

        d = math.sqrt(x * x + y * y)
        error = d - self.target_d # P
        errori = self.intg + error / 120 # I
        errord = error * 120 # D

        self.intg = errori

        Ps = 1
        Is = 1
        Ds = 1
        Pv = 1
        Iv = 1
        Dv = 1
        drift = 0.5
        '''
        ori = np.arctan2(0 - y, 0 - x)
        # the first part is used to keep drawing circle
        # the second part is used to keep drifting
        ctrl_servo = 0.4
        ctrl_speed = 5.0

        if(self.safe == 0 and v <= 5.5):
            self.ctrl.control.mode = 3 # speed control
            self.ctrl.control.speed = ctrl_speed
            self.ctrl.control.servo = ctrl_servo
        else:
            self.ctrl.control.mode = 1 # brake contol
            self.ctrl.control.brake = 20
            self.ctrl.control.servo = 0
            self.safe = 1

        self.drive_pub.publish(self.ctrl)

if __name__ == '__main__':
    rospy.init_node('PIDdrift')
    pid_drift = PIDdrift()
    rospy.spin()
