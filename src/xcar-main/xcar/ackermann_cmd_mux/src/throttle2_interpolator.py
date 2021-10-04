#!/usr/bin/env python
import rospy

from std_msgs.msg import Float64,Int64

# import some utils.
import numpy as np
import copy as copy

class InterpolateThrottle:
    def __init__(self):

        # Allow our topics to be dynamic.
        # topic name
        self.rpm_input_topic   = '/vesc/commands/motor/raw_speed'
        self.rpm_output_topic  = '/vesc/commands/motor/speed'

        self.servo_input_topic   = '/vesc/commands/servo/raw_position'
        self.servo_output_topic  = '/vesc/commands/servo/position'

        self.current_input_topic   = '/vesc/commands/motor/raw_current'
        self.current_output_topic  = '/vesc/commands/motor/current'

        self.brake_input_topic   = '/vesc/commands/motor/raw_brake'
        self.brake_output_topic  = '/vesc/commands/motor/brake'


        #param
        self.max_acceleration = rospy.get_param('/vesc/max_acceleration')
        self.max_rpm = rospy.get_param('/vesc/vesc_driver/speed_max')
        self.min_rpm = rospy.get_param('/vesc/vesc_driver/speed_min')
        self.throttle_smoother_rate = rospy.get_param('/vesc/throttle_smoother_rate')
        self.speed_to_erpm_gain = rospy.get_param('/vesc/speed_to_erpm_gain')

        self.max_servo_speed = rospy.get_param('/vesc/max_servo_speed')
        self.steering_angle_to_servo_gain = rospy.get_param('/vesc/steering_angle_to_servo_gain')
        self.servo_smoother_rate = rospy.get_param('/vesc/servo_smoother_rate')
        self.max_servo = rospy.get_param('/vesc/vesc_driver/servo_max')
        self.min_servo = rospy.get_param('/vesc/vesc_driver/servo_min')

        self.max_current = rospy.get_param('/vesc/vesc_driver/current_max')
        self.min_current = rospy.get_param('/vesc/vesc_driver/current_min')

        self.max_brake = rospy.get_param('/vesc/vesc_driver/brake_max')
        self.min_brake = rospy.get_param('/vesc/vesc_driver/brake_min')

        # Variables
        self.mode = 0
        self.last_rpm = 0
        self.desired_rpm = self.last_rpm
        
        self.last_servo = rospy.get_param('/vesc/steering_angle_to_servo_offset')
        self.desired_servo_position = self.last_servo

        self.desired_brake = 30
        self.desired_current = 0

        # Create topic subscribers and publishers
        rospy.Subscriber('/vesc/commands/motor/mode',Int64,self._process_mode)
        rospy.Subscriber(self.brake_input_topic, Float64, self._process_brake_command)
        rospy.Subscriber(self.current_input_topic, Float64, self._process_current_command)
        rospy.Subscriber(self.rpm_input_topic, Float64, self._process_throttle_command)
        rospy.Subscriber(self.servo_input_topic, Float64, self._process_servo_command)

        self.brake_output = rospy.Publisher(self.brake_output_topic, Float64,queue_size=1)
        self.current_output = rospy.Publisher(self.current_output_topic, Float64,queue_size=1)
        self.rpm_output = rospy.Publisher(self.rpm_output_topic, Float64,queue_size=1)
        self.servo_output = rospy.Publisher(self.servo_output_topic, Float64,queue_size=1)
    
        

        self.max_delta_servo = abs(self.steering_angle_to_servo_gain * self.max_servo_speed / self.servo_smoother_rate)
        #rospy.Timer(rospy.Duration(1.0/self.servo_smoother_rate), self._publish_servo_command)

        self.max_delta_rpm = abs(self.speed_to_erpm_gain * self.max_acceleration / self.throttle_smoother_rate)
        #rospy.Timer(rospy.Duration(1.0/self.max_delta_rpm), self._publish_throttle_command)
        
        rospy.Timer(rospy.Duration(1.0/self.servo_smoother_rate),self._publish_command)

        # run the node
        
        self._run()

    def _publish_command(self, evt):
        mode = self.mode
        desired_delta = self.desired_servo_position-self.last_servo
        clipped_delta = max(min(desired_delta, self.max_delta_servo), -self.max_delta_servo)
        smoothed_servo = self.last_servo + clipped_delta
        self.last_servo = smoothed_servo         
        self.servo_output.publish(Float64(smoothed_servo))
        if mode == 1:
            smoothed_brake = self.desired_brake        
            self.brake_output.publish(Float64(smoothed_brake))
            self.last_rpm = 0
        elif mode == 2:
            smoothed_current = self.desired_current        
            self.current_output.publish(Float64(smoothed_current))
        elif mode == 3:
            desired_delta = self.desired_rpm-self.last_rpm
            clipped_delta = max(min(desired_delta, self.max_delta_rpm), -self.max_delta_rpm)
            smoothed_rpm = self.last_rpm + clipped_delta
            self.last_rpm = smoothed_rpm         
            # print self.desired_rpm, smoothed_rpm
            self.rpm_output.publish(Float64(smoothed_rpm))

        # Keep the node alive
    def _run(self):
        rospy.spin()
    
    def _process_mode(self,msg):
        self.mode = msg.data

    def _process_brake_command(self,msg):
        input_brake = msg.data
        input_brake = min(max(input_brake, self.min_brake), self.max_brake)
        self.desired_brake = input_brake
    
    def _process_current_command(self,msg):
        input_current = msg.data
        input_current = min(max(input_current, self.min_current), self.max_current)
        self.desired_current = input_current
    
    def _process_throttle_command(self,msg):
        input_rpm = msg.data
        # Do some sanity clipping
        input_rpm = min(max(input_rpm, self.min_rpm), self.max_rpm)
        self.desired_rpm = input_rpm
    
    def _process_servo_command(self,msg):
        input_servo = msg.data
        # Do some sanity clipping
        input_servo = min(max(input_servo, self.min_servo), self.max_servo)
        # set the target servo position
        self.desired_servo_position = input_servo
    '''        
    def _publish_brake_command(self):
        smoothed_brake = self.desired_brake        
        self.brake_output.publish(Float64(smoothed_brake))
    
    def _publish_current_command(self):
        smoothed_current = self.desired_current        
        self.current_output.publish(Float64(smoothed_current))

    def _publish_throttle_command(self):
        desired_delta = self.desired_rpm-self.last_rpm
        clipped_delta = max(min(desired_delta, self.max_delta_rpm), -self.max_delta_rpm)
        smoothed_rpm = self.last_rpm + clipped_delta
        self.last_rpm = smoothed_rpm         
        # print self.desired_rpm, smoothed_rpm
        self.rpm_output.publish(Float64(smoothed_rpm))
        
    def _publish_servo_command(self):
        desired_delta = self.desired_servo_position-self.last_servo
        clipped_delta = max(min(desired_delta, self.max_delta_servo), -self.max_delta_servo)
        smoothed_servo = self.last_servo + clipped_delta
        self.last_servo = smoothed_servo         
        self.servo_output.publish(Float64(smoothed_servo))
    '''

    

# Boilerplate node spin up. 
if __name__ == '__main__':
    try:
        rospy.init_node('Throttle_Interpolator')
        p = InterpolateThrottle()
    except rospy.ROSInterruptException:
        pass
