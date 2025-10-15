#!/usr/bin/env python3

import os
import sys
import rospy
import numpy as np
import time
from std_msgs.msg import Float32MultiArray, Header
from geometry_msgs.msg import Pose, Point, Twist
from nav_msgs.msg import Odometry
import tf.transformations as tf
from rosgraph_msgs.msg import Clock
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import GetModelState
from std_msgs.msg import Bool

class WheelSpeedController:
    def __init__(self):
        rospy.init_node("wheel_speed_controller", anonymous=False)
        
        self.wait_for_spawn("robot",5.0)
        
        rospy.sleep(1.0)
        
        # Parameters for the PID controller
        self.Kp = rospy.get_param("~Kp", 1.95)
        self.Ki = rospy.get_param("~Ki", 4.8)
        self.Kd = rospy.get_param("~Kd", 0.0)
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.075)
        self.lx = rospy.get_param('~lx', 0.28)
        self.ly = rospy.get_param('~ly', 0.11)
        
        self.use_world_transform = rospy.get_param("~world_tf", True)
        
        self.vy_adjust_factor = 1.2
        
        self.e_ant = [0,0,0,0]
        self.control_outputs_ant = [0, 0, 0, 0]
        self.u = [0,0,0]
        
        self.yaw = 0.0
        
        self.first_pass = True
        
        # Setpoint for the wheel speed
        self.setpoint = [0.0, 0.0, 0.0, 0.0]
        
        #Calculo constantes para el controlador
        self.Ts = 1.0/90.909       
        
        self.A0 = self.Kp + self.Ki * self.Ts / 2 + 2 * self.Kd / self.Ts
        self.A1 = -self.Kp + self.Ki * self.Ts / 2 - 2 * self.Kd / self.Ts
        self.A2 = self.Kd / self.Ts  # Additional term needed for proper derivative calculation
        
        # Publisher for the control output
        self.control_publisher = rospy.Publisher("gazebo_wheel_cmd_sim", Float32MultiArray, queue_size=10)
        
        # Subscriber for the wheel speed
        rospy.Subscriber("/processed_wheel_speed", Float32MultiArray, self.control_callback)
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/reset_robot", Bool, self.reset_robot_callback)
        
        #rospy.Subscriber("/wheel_setpoint", Float32MultiArray, self.setpoint_callback) use only when testing the PID, and comment setpoint from odom callback
        
        self.stop()
        
        rospy.sleep(1.0)
        
        self.reset_robot()
        
        
    def get_inv_Jacobian(self,th):
        th1 = th + np.pi/4
        r2 = np.sqrt(2)
        J_inv = np.array([[r2 * np.cos(th1) , r2 * np.sin(th1) * self.vy_adjust_factor, -(self.lx + self.ly)],
                          [r2 * np.sin(th1) ,-r2 * np.cos(th1) * self.vy_adjust_factor, -(self.lx + self.ly)],
                          [r2 * np.cos(th1) , r2 * np.sin(th1) * self.vy_adjust_factor,  (self.lx + self.ly)],
                          [r2 * np.sin(th1) ,-r2 * np.cos(th1) * self.vy_adjust_factor,  (self.lx + self.ly)]])
        return J_inv
    
    
    def wait_for_spawn(self, robot_name, timeout):
        
        rospy.loginfo(f"Waiting for '{robot_name}' to spawn in Gazebo...")
        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            try:
                msg = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=1.0)
                if robot_name in msg.name:
                    rospy.loginfo(f"Robot '{robot_name}' detected in Gazebo")
                    return True
            except rospy.ROSException:
                pass  # timed out, retry

            if timeout and (rospy.Time.now() - start_time).to_sec() > timeout:
                rospy.logwarn(f"Timeout waiting for '{robot_name}'. Proceeding anyway...")
                return False
    
    def reset_robot(self):
        self.setpoint = [0.0, 0.0, 0.0, 0.0]
        self.u = [0,0,0]
        self.stop()
        rospy.sleep(0.25)
           
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

            model_state = ModelState()
            model_state.model_name = 'robot'
            model_state.pose = Pose()
            model_state.pose.position.x = 0.0
            model_state.pose.position.y = 0.0
            model_state.pose.position.z = 0.0
            model_state.pose.orientation.x = 0.0
            model_state.pose.orientation.y = 0.0
            model_state.pose.orientation.z = 0.7071068
            model_state.pose.orientation.w = 0.7071068    #1.0

            model_state.twist = Twist()
            model_state.twist.linear.x = 0.0
            model_state.twist.linear.y = 0.0
            model_state.twist.linear.z = 0.0
            model_state.twist.angular.x = 0.0
            model_state.twist.angular.y = 0.0
            model_state.twist.angular.z = 0.0

            model_state.reference_frame = 'world'

            resp = set_state(model_state)
            
            if resp.success:
                rospy.loginfo("Robot position set succesfully")
            else:
                rospy.logerr("Failed to set model state: %s", resp.status_message)

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
         
    def reset_robot_callback(self, msg):
        self.reset_robot()
    
    def odom_callback(self, msg):
        # Orientation quaternion
        q = msg.pose.pose.orientation
        quaternion = (q.x, q.y, q.z, q.w)

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        roll, pitch, yaw = tf.euler_from_quaternion(quaternion)
        
        angle = yaw if self.use_world_transform else 0.0
            
        J_inv = self.get_inv_Jacobian(angle)
        self.setpoint = np.dot(J_inv,self.u)/self.wheel_radius
    
    def cmd_vel_callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z
        self.u = np.array([[vx],[vy],[wz]])
    
    def setpoint_callback(self, msg):
        # Update the setpoints for each PID controller
        new_setpoints = msg.data

        # Ensure we have the correct number of setpoints
        if len(new_setpoints) != 4:
            #rospy.logwarn("Received setpoint data does not match expected number of wheels (4).")
            return

        self.setpoint = new_setpoints
    
    def control_callback(self, msg):
        # Read current wheel speeds
        current_speeds = [msg.data[0], msg.data[1], msg.data[2], msg.data[3]]
        
        control_outputs = [0,0,0,0]
        
        # Compute control outputs for each wheel
        for i in range(4):
            error = self.setpoint[i] - current_speeds[i]
            # Compute control output with the Tustin method
            
            if self.setpoint[i] == 0:
                control_outputs[i] = 0
            else:
                control_outputs[i] = self.control_outputs_ant[i] + self.A0 * error + self.A1 * self.e_ant[i] + self.A2 * (error - self.e_ant[i])
            
            if (control_outputs[i]>=15.0):
                control_outputs[i] = 15.0
            if (control_outputs[i]<=-15.0):
                control_outputs[i] = -15.0
            
            # Update previous values
            self.control_outputs_ant[i] = control_outputs[i]
            self.e_ant[i] = error
            
        # Create a message for the control outputs
        control_msg = Float32MultiArray(data=control_outputs)

        # Publish the control outputs
        self.control_publisher.publish(control_msg)
        
               
    def stop(self):
        stop_msg = Float32MultiArray(data=[0.0, 0.0, 0.0, 0.0])
        # Publish the stop message
        self.control_publisher.publish(stop_msg)

if __name__ == "__main__":
    try:
        node = WheelSpeedController()
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
    finally:
        sys.exit()
