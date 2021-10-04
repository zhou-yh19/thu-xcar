#!/usr/bin/env python
# BEGIN ALL
import rospy,cv2,cv_bridge, numpy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, Bool
from ackermann_msgs.msg import AckermannDriveStamped
from vesc_msgs.msg import VescCtrlStamped
import math
from vesc_msgs.msg import VescStateStamped

class ViconDriver:
    def __init__(self):
		self.state_sub = rospy.Subscriber('/vesc/sensors/servo_position_command',Float64,self.state_callback)
		self.vicon_sub = rospy.Subscriber('/vrpn_client_node/racecar/pose',PoseStamped,self.vicon_callback)
		#self.robo_sub = rospy.Subscriber('/vrpn_client_node/robomaster/pose',PoseStamped,self.robo_callback)
		self.cmd_vel_pub = rospy.Publisher('/vesc/ctrl', VescCtrlStamped, queue_size=1)
		self.ctrl = VescCtrlStamped()
		self.last_d = 0
		self.intg = 0
		self.servo = 0.0
		self.max_x = -100
		self.min_x = +100
		self.max_y = -100
		self.min_y = +100
		self.robo_x = 0 
		self.robo_y = 0

    def state_callback(self,msg):
		self.servo = msg.data

    def robo_callback(self,msg):
		self.robo_x = msg.pose.position.x
		self.robo_y = msg.pose.position.y

    def vicon_callback(self,msg):
		x = msg.pose.position.x
		y = msg.pose.position.y
		if(x >= self.max_x):
			self.max_x = x
		if(x <= self.min_x):
			self.min_x = x
		if(y >= self.max_y):
			self.max_y = y
		if(y <= self.min_y):
			self.min_y = y
		'''
		if(abs(y)<=2):
			d = abs(x)
		if(y>2):
			d = math.sqrt(x * x + (y - 2) * (y - 2))
		if(y<-2):
			d = math.sqrt(x * x + (y + 2) * (y + 2))
		'''
		d = math.sqrt(x * x + 0.4 * y * y)
		#d = math.sqrt((x - self.robo_x) * (x - self.robo_x) + (y - self.robo_y) * (y - self.robo_y))
		dot_d = (d - self.last_d)*120
		#print('x_max = ',self.max_x)
		#print('x_min = ',self.min_x)
		#print('y_max = ',self.max_y)
		#print('y_min = ',self.min_y)
		print('d = ',d)
		e = (dot_d + 0.3 * math.tanh(d-1.5))
		self.intg = self.intg + e/120 
		w = 100 * e + 10 * self.intg
		self.last_d = d
		#ctrl_servo = -0.0275 + 0.2 + w/120
		ctrl_servo = w/120
		ctrl_speed = 1
	
		self.ctrl.control.mode = 3 #speed control        
		self.ctrl.control.speed = ctrl_speed
		self.ctrl.control.servo = ctrl_servo
	
		self.cmd_vel_pub.publish(self.ctrl)
        

rospy.init_node('Auto')
vicondriver = ViconDriver()
rospy.spin()

