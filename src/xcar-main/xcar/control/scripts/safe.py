#!/usr/bin/env python
# BEGIN ALL
import rospy,cv2,cv_bridge, numpy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, Bool
from ackermann_msgs.msg import AckermannDriveStamped

class LaserDriver:
    def __init__(self):
        self.laser_sub = rospy.Subscriber('/scan',LaserScan,self.laser_callback)
        self.cmd_vel_pub = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/safety', AckermannDriveStamped, queue_size=1)
        self.ack = AckermannDriveStamped()
    


    def laser_callback(self,msg):	   
        self.ack.drive.speed = 100
        self.ack.drive.steering_angle = -0.04021
            
        self.cmd_vel_pub.publish(self.ack)

rospy.init_node('Auto')
laserdriver = LaserDriver()
rospy.spin()

