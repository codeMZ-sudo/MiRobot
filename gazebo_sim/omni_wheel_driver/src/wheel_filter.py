#!/usr/bin/env python3

import os
import sys
import rospy
import time
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
import tf.transformations
from tf.transformations import quaternion_from_euler
import math
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64
from rosgraph_msgs.msg import Clock

class Driver_Sensors:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('driver_sensors', anonymous=True)
        
        self.unfiltered_angular_speeds = [0.0, 0.0, 0.0, 0.0]
        self.filtered_angular_speeds = [0.0, 0.0, 0.0, 0.0]
        self.filtered_angular_speeds_prev = [0.0, 0.0, 0.0, 0.0]
        self.n = 0  # Number of pulses to wait before computing speed
        self.max_delta_threshold = 600  # Threshold for sudden changes
        self.alpha = 0.2
        self.sim_time = rospy.Time.now()
        self.previous_time = rospy.Time.now()
        self.current_values = []
        self.previous_values = []


        # Publisher for the control output
        self.filtered_wheels_pub = rospy.Publisher("/processed_wheel_speed", Float32MultiArray, queue_size=10)
        rospy.Subscriber("/gazebo_wheel_speed_sim", Float32MultiArray, self.wheel_speed_callback)
        
        self.use_filter = rospy.get_param('use_filter', True)
        
        rospy.Subscriber('/clock', Clock, self.clock_callback)             
        if not rospy.get_param('/use_sim_time', False):
            rospy.set_param('/use_sim_time', True)
        

    def clock_callback(self, msg):
        self.sim_time = msg.clock
        
    def wheel_speed_callback(self, msg):
        if self.use_filter:
            self.unfiltered_angular_speeds = msg.data
            self.compute_speed_filtered()
            wheel_speed = self.filtered_angular_speeds            
        else:
            wheel_speed = msg.data
        self.filtered_wheels_pub.publish(Float32MultiArray(data=wheel_speed))
        
        
    def compute_speed_filtered(self):
        current_time = self.sim_time				# Cambiar por el rospy.time.now()
        current_values = self.unfiltered_angular_speeds
        delta_time = (current_time - self.previous_time).to_sec()
        if delta_time > 0.005:
            for i in range(4):
                self.filtered_angular_speeds[i] = self.alpha * current_values[i] + (1 - self.alpha) * self.filtered_angular_speeds[i]           
            self.previous_time = current_time
        return self.filtered_angular_speeds

if __name__ == "__main__":
    try:
        sensor = Driver_Sensors()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

