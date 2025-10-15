#!/usr/bin/env python3

import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist, PoseStamped, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion
import tf


class DWAConfig:
    """
    Configuration parameters for DWA algorithm.
    All parameters are adjustable based on your robot's specifications.
    """
    def __init__(self):
        # Robot specifications (ADJUST THESE FOR YOUR ROBOT)
        # Maximum velocities [m/s and rad/s]
        self.max_vel_x = rospy.get_param('~max_vel_x', 0.35)  # Forward/backward
        self.max_vel_y = rospy.get_param('~max_vel_y', 0.35)  # Lateral (mecanum specific)
        self.max_vel_theta = rospy.get_param('~max_vel_theta', 1.0)  # Rotational
        
        # Minimum velocities [m/s and rad/s]
        self.min_vel_x = rospy.get_param('~min_vel_x', -0.35)
        self.min_vel_y = rospy.get_param('~min_vel_y', -0.35)
        self.min_vel_theta = rospy.get_param('~min_vel_theta', -1.0)
        
        # Acceleration limits [m/s^2 and rad/s^2]
        self.max_accel_x = rospy.get_param('~max_accel_x', 0.5)
        self.max_accel_y = rospy.get_param('~max_accel_y', 0.5)
        self.max_accel_theta = rospy.get_param('~max_accel_theta', 1.5)
        
        # Deceleration limits [m/s^2 and rad/s^2]
        self.max_decel_x = rospy.get_param('~max_decel_x', -0.5)
        self.max_decel_y = rospy.get_param('~max_decel_y', -0.5)
        self.max_decel_theta = rospy.get_param('~max_decel_theta', -1.5)
        
        # Velocity resolution for sampling [m/s and rad/s]
        self.vel_resolution_x = rospy.get_param('~vel_resolution_x', 0.05)
        self.vel_resolution_y = rospy.get_param('~vel_resolution_y', 0.05)
        self.vel_resolution_theta = rospy.get_param('~vel_resolution_theta', 0.1)
        
        # Prediction time horizon [seconds]
        self.predict_time = rospy.get_param('~predict_time', 2.0)
        
        # Time step for trajectory prediction [seconds]
        self.dt = rospy.get_param('~dt', 0.1)
        
        # Cost function weights (adjust these to tune behavior)
        self.heading_cost_gain = rospy.get_param('~heading_cost_gain', 1.0)
        self.distance_cost_gain = rospy.get_param('~distance_cost_gain', 1.0)
        self.velocity_cost_gain = rospy.get_param('~velocity_cost_gain', 0.5)
        self.obstacle_cost_gain = rospy.get_param('~obstacle_cost_gain', 2.0)
        
        # Robot geometry
        self.robot_radius = rospy.get_param('~robot_radius', 0.25)  # [m]
        
        # Safety distance from obstacles [m]
        self.obstacle_margin = rospy.get_param('~obstacle_margin', 0.05)
        
        # Goal tolerance [m and rad]
        self.goal_tolerance_xy = rospy.get_param('~goal_tolerance_xy', 0.1)
        self.goal_tolerance_theta = rospy.get_param('~goal_tolerance_theta', 0.1)


class DWAPlannerMecanum:
    """
    Dynamic Window Approach planner specifically designed for mecanum wheel robots.
    
    Key differences from differential drive DWA:
    - 3D velocity space (vx, vy, omega) instead of 2D (v, omega)
    - Omnidirectional motion capabilities
    - Independent X and Y velocity control
    """
    
    def __init__(self):
        rospy.init_node('dwa_planner_mecanum', anonymous=False)
        
        # Load configuration
        self.config = DWAConfig()
        
        # State variables
        self.current_pose = None  # [x, y, theta]
        self.current_vel = np.array([0.0, 0.0, 0.0])  # [vx, vy, omega]
        self.goal_pose = None  # [x, y, theta]
        self.obstacle_points = []  # List of [x, y] obstacle positions
        
        # ROS Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.local_path_pub = rospy.Publisher('/local_path', Path, queue_size=1)
        
        # ROS Subscribers
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/goal', PoseStamped, self.goal_callback)
        
        # TF listener
        self.tf_listener = tf.TransformListener()
        
        # Control loop rate
        self.rate = rospy.Rate(10)  # 10 Hz
        
        rospy.loginfo("DWA Planner for Mecanum Wheel Robot initialized")
    
    def odom_callback(self, msg):
        """
        Update current robot pose and velocity from odometry.
        """
        # Extract position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Extract orientation (convert quaternion to euler)
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, 
                          orientation_q.z, orientation_q.w]
        (_, _, theta) = euler_from_quaternion(orientation_list)
        
        self.current_pose = np.array([x, y, theta])
        
        # Extract velocities (in robot frame)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        omega = msg.twist.twist.angular.z
        
        self.current_vel = np.array([vx, vy, omega])
    
    def scan_callback(self, msg):
        """
        Process laser scan data to extract obstacle positions.
        Converts laser scan ranges to Cartesian coordinates in the robot frame.
        """
        if self.current_pose is None:
            return
        
        self.obstacle_points = []
        
        # Process each laser scan point
        for i, r in enumerate(msg.ranges):
            # Skip invalid readings
            if r < msg.range_min or r > msg.range_max or np.isinf(r) or np.isnan(r):
                continue
            
            # Calculate angle of this reading
            angle = msg.angle_min + i * msg.angle_increment
            
            # Convert to Cartesian coordinates (robot frame)
            x_robot = r * np.cos(angle)
            y_robot = r * np.sin(angle)
            
            # Transform to global frame
            x_global = self.current_pose[0] + x_robot * np.cos(self.current_pose[2]) - \
                      y_robot * np.sin(self.current_pose[2])
            y_global = self.current_pose[1] + x_robot * np.sin(self.current_pose[2]) + \
                      y_robot * np.cos(self.current_pose[2])
            
            self.obstacle_points.append([x_global, y_global])
    
    def goal_callback(self, msg):
        """
        Set new goal pose from RViz or other sources.
        """
        x = msg.pose.position.x
        y = msg.pose.position.y
        
        orientation_q = msg.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, 
                          orientation_q.z, orientation_q.w]
        (_, _, theta) = euler_from_quaternion(orientation_list)
        
        self.goal_pose = np.array([x, y, theta])
        rospy.loginfo(f"New goal received: x={x:.2f}, y={y:.2f}, theta={theta:.2f}")
    
    def calculate_min_obstacle_distance(self, vx, vy, omega):
        """
        Calculate minimum distance to obstacles along a trajectory.
        
        This is used for the obstacle avoidance window (Va) calculation.
        Simulates the trajectory and finds the closest approach to any obstacle.
        
        Args:
            vx, vy, omega: Velocity components
            
        Returns:
            float: Minimum distance to any obstacle along trajectory [m]
        """
        if len(self.obstacle_points) == 0:
            return float('inf')
        
        # Simulate trajectory
        trajectory = self.predict_trajectory(vx, vy, omega)
        
        min_dist = float('inf')
        for point in trajectory:
            for obstacle in self.obstacle_points:
                dist = np.hypot(point[0] - obstacle[0], point[1] - obstacle[1])
                min_dist = min(min_dist, dist)
        
        return min_dist
    
    def calculate_safe_velocity_limit(self, velocity, deceleration):
        """
        Calculate maximum safe velocity that allows stopping before collision.
        
        Based on: v_safe = sqrt(2 * dist * decel)
        From DWA paper equation (4): Va = {(v,ω)|v ≤ √(2·dist(v,ω)·v̇_l), ω ≤ √(2·dist(v,ω)·ω̇_l)}
        
        Args:
            velocity: Current velocity component
            deceleration: Maximum deceleration (positive value)
            
        Returns:
            float: Maximum safe velocity limit
        """
        # For stopped or very slow motion, return small positive value
        if abs(velocity) < 0.01:
            return 0.05
        
        # Calculate minimum distance along this velocity direction
        # We need to check the actual trajectory, but for efficiency,
        # we'll use a simplified approach based on current velocity direction
        return velocity  # Will be constrained by Va in calculate_dynamic_window
    
    def calculate_dynamic_window(self):
        """
        Calculate the dynamic window based on current velocity and robot constraints.
        
        For mecanum wheels, we have a 3D velocity space: (vx, vy, omega)
        This now includes the obstacle avoidance window Va.
        
        Returns Vr = Vs ∩ Vd ∩ Va
        
        Returns:
            tuple: (vx_min, vx_max, vy_min, vy_max, omega_min, omega_max)
        """
        # 1. Velocity space based on robot max velocities (Vs)
        Vs = [self.config.min_vel_x, self.config.max_vel_x,
              self.config.min_vel_y, self.config.max_vel_y,
              self.config.min_vel_theta, self.config.max_vel_theta]
        
        # 2. Dynamic window based on acceleration limits (Vd)
        Vd = [
            self.current_vel[0] + self.config.max_decel_x * self.config.dt,
            self.current_vel[0] + self.config.max_accel_x * self.config.dt,
            self.current_vel[1] + self.config.max_decel_y * self.config.dt,
            self.current_vel[1] + self.config.max_accel_y * self.config.dt,
            self.current_vel[2] + self.config.max_decel_theta * self.config.dt,
            self.current_vel[2] + self.config.max_accel_theta * self.config.dt
        ]
        
        # 3. Obstacle avoidance window (Va) - NEW!
        # Calculate safe velocity limits based on obstacle distances
        Va = self.calculate_obstacle_avoidance_window()
        
        # 4. Intersection: Vr = Vs ∩ Vd ∩ Va
        dw = [
            max(Vs[0], Vd[0], Va[0]),
            min(Vs[1], Vd[1], Va[1]),
            max(Vs[2], Vd[2], Va[2]),
            min(Vs[3], Vd[3], Va[3]),
            max(Vs[4], Vd[4], Va[4]),
            min(Vs[5], Vd[5], Va[5])
        ]
        
        return dw
    
    def calculate_obstacle_avoidance_window(self):
        """
        Calculate the obstacle avoidance window Va.
        
        From DWA paper equation (4):
        Va = {(v,ω)|v ≤ √(2·dist(v,ω)·v̇_l)^(1/2), ω ≤ √(2·dist(v,ω)·ω̇_l)^(1/2)}
        
        This ensures only velocities that allow stopping before collision are admissible.
        
        Returns:
            list: [vx_min, vx_max, vy_min, vy_max, omega_min, omega_max]
        """
        if len(self.obstacle_points) == 0:
            # No obstacles, return unrestricted window
            return [
                self.config.min_vel_x, self.config.max_vel_x,
                self.config.min_vel_y, self.config.max_vel_y,
                self.config.min_vel_theta, self.config.max_vel_theta
            ]
        
        # Sample several velocity directions to find safe limits
        # For mecanum robot, we need to consider combined linear velocity
        
        # Calculate safe limits for different velocity combinations
        safe_vx_pos = self.calculate_safe_vel_for_direction(1, 0, 0)
        safe_vx_neg = self.calculate_safe_vel_for_direction(-1, 0, 0)
        safe_vy_pos = self.calculate_safe_vel_for_direction(0, 1, 0)
        safe_vy_neg = self.calculate_safe_vel_for_direction(0, -1, 0)
        safe_omega_pos = self.calculate_safe_vel_for_direction(0, 0, 1)
        safe_omega_neg = self.calculate_safe_vel_for_direction(0, 0, -1)
        
        Va = [
            -safe_vx_neg,  # min vx
            safe_vx_pos,   # max vx
            -safe_vy_neg,  # min vy
            safe_vy_pos,   # max vy
            -safe_omega_neg,  # min omega
            safe_omega_pos    # max omega
        ]
        
        return Va
    
    def calculate_safe_vel_for_direction(self, vx_dir, vy_dir, omega_dir):
        """
        Calculate maximum safe velocity in a specific direction.
        
        Uses the formula: v_safe = sqrt(2 * dist * decel)
        
        Args:
            vx_dir, vy_dir, omega_dir: Direction vector (normalized to 1 or 0)
            
        Returns:
            float: Maximum safe velocity in this direction
        """
        # Determine which velocity component and deceleration to use
        if vx_dir != 0:
            test_vel = vx_dir * self.config.max_vel_x
            decel = abs(self.config.max_decel_x)
        elif vy_dir != 0:
            test_vel = vy_dir * self.config.max_vel_y
            decel = abs(self.config.max_decel_y)
        else:  # omega_dir != 0
            test_vel = omega_dir * self.config.max_vel_theta
            decel = abs(self.config.max_decel_theta)
        
        # Get minimum distance to obstacles in this direction
        dist = self.calculate_min_obstacle_distance(
            vx_dir * abs(test_vel) if vx_dir != 0 else 0,
            vy_dir * abs(test_vel) if vy_dir != 0 else 0,
            omega_dir * abs(test_vel) if omega_dir != 0 else 0
        )
        
        # Account for robot radius and safety margin
        dist = max(0, dist - self.config.robot_radius - self.config.obstacle_margin)
        
        # Calculate safe velocity: v = sqrt(2 * dist * decel)
        safe_vel = math.sqrt(2 * dist * decel)
        
        # Clamp to reasonable limits
        if vx_dir != 0:
            safe_vel = min(safe_vel, abs(self.config.max_vel_x))
        elif vy_dir != 0:
            safe_vel = min(safe_vel, abs(self.config.max_vel_y))
        else:
            safe_vel = min(safe_vel, abs(self.config.max_vel_theta))
        
        return safe_vel
    
    def predict_trajectory(self, vx, vy, omega):
        """
        Predict robot trajectory for given velocities.
        
        For mecanum wheels, the kinematic model is:
        x_dot = vx * cos(theta) - vy * sin(theta)
        y_dot = vx * sin(theta) + vy * cos(theta)
        theta_dot = omega
        
        Args:
            vx: Linear velocity in X direction (robot frame) [m/s]
            vy: Linear velocity in Y direction (robot frame) [m/s]
            omega: Angular velocity [rad/s]
        
        Returns:
            np.array: Predicted trajectory [[x1, y1, theta1], [x2, y2, theta2], ...]
        """
        trajectory = []
        x, y, theta = self.current_pose
        
        time = 0
        while time <= self.config.predict_time:
            # Update position using mecanum kinematics
            x += (vx * np.cos(theta) - vy * np.sin(theta)) * self.config.dt
            y += (vx * np.sin(theta) + vy * np.cos(theta)) * self.config.dt
            theta += omega * self.config.dt
            
            # Normalize theta to [-pi, pi]
            theta = np.arctan2(np.sin(theta), np.cos(theta))
            
            trajectory.append([x, y, theta])
            time += self.config.dt
        
        return np.array(trajectory)
    
    def calculate_obstacle_cost(self, trajectory):
        """
        Calculate cost based on distance to nearest obstacle.
        
        NOTE: With Va implemented, this should rarely return inf since
        unsafe velocities are already filtered out.
        
        Args:
            trajectory: Predicted trajectory points
        
        Returns:
            float: Obstacle cost (0 = far from obstacles, inf = collision)
        """
        min_distance = float('inf')
        
        for point in trajectory:
            for obstacle in self.obstacle_points:
                dist = np.hypot(point[0] - obstacle[0], point[1] - obstacle[1])
                min_distance = min(min_distance, dist)
                
                # Check for collision (should be rare now with Va)
                if dist < self.config.robot_radius + self.config.obstacle_margin:
                    #print("inf")
                    return float('inf')  # Collision detected
        
        # Return inverse distance (closer obstacles = higher cost)
        if min_distance < float('inf'):
            return 1.0 / (min_distance + 0.1)  # Add small value to avoid division by zero
        else:
            return 0.0
    
    def calculate_heading_cost(self, trajectory):
        """
        Calculate cost based on alignment with goal direction.
        
        Lower cost when robot heading is aligned with goal direction.
        
        Args:
            trajectory: Predicted trajectory points
        
        Returns:
            float: Heading cost (0 = perfectly aligned, higher = misaligned)
        """
        if self.goal_pose is None:
            return 0.0
        
        # Use the final point of trajectory
        final_pose = trajectory[-1]
        
        theta = final_pose[2] - self.goal_pose[2] 
        theta = np.arctan2(np.sin(theta), np.cos(theta))
        heading_error = abs(theta)/np.pi
        
        return heading_error
    
    def calculate_distance_cost(self, trajectory):
        """
        Calculate cost based on distance to goal.
        
        Prefers trajectories that get closer to the goal.
        
        Args:
            trajectory: Predicted trajectory points
        
        Returns:
            float: Distance cost (lower = closer to goal)
        """
        if self.goal_pose is None:
            return 0.0
        
        # Use the final point of trajectory
        final_pose = trajectory[-1]
        
        # Calculate distance to goal
        distance = np.hypot(self.goal_pose[0] - final_pose[0],
                           self.goal_pose[1] - final_pose[1])
        
        return distance
    
    def calculate_velocity_cost(self, vx, vy, omega):
        """
        Calculate cost based on velocity magnitude.
        
        Encourages higher velocities (faster movement).
        
        Args:
            vx, vy, omega: Velocity components
        
        Returns:
            float: Velocity cost (higher velocity = lower cost)
        """
        # Calculate linear velocity magnitude
        linear_vel = np.hypot(vx, vy)
        
        # Normalize by max velocity
        max_linear_vel = np.hypot(self.config.max_vel_x, self.config.max_vel_y)
        
        # Return inverse (higher velocity = lower cost)
        return (max_linear_vel - linear_vel) / max_linear_vel
    
    def evaluate_trajectory(self, vx, vy, omega):
        """
        Evaluate a trajectory using the complete cost function.
        
        Total cost = w1*heading_cost + w2*distance_cost + w3*velocity_cost + w4*obstacle_cost
        
        Args:
            vx, vy, omega: Velocity components
        
        Returns:
            tuple: (total_cost, trajectory)
        """
        # Predict trajectory
        trajectory = self.predict_trajectory(vx, vy, omega)
        
        # Calculate individual costs
        heading_cost = self.calculate_heading_cost(trajectory)
        distance_cost = self.calculate_distance_cost(trajectory)
        velocity_cost = self.calculate_velocity_cost(vx, vy, omega)
        obstacle_cost = self.calculate_obstacle_cost(trajectory)
        
        # If collision detected, return infinite cost
        if obstacle_cost == float('inf'):
            return float('inf'), trajectory
        
        # Weighted sum of costs
        total_cost = (
            self.config.heading_cost_gain * heading_cost +
            self.config.distance_cost_gain * distance_cost +
            self.config.velocity_cost_gain * velocity_cost +
            self.config.obstacle_cost_gain * obstacle_cost
        )
        
        return total_cost, trajectory
    
    def dwa_control(self):
        """
        Main DWA control algorithm.
        
        Samples velocities within the dynamic window and selects the best one.
        Now with Va, only safe velocities are sampled.
        
        Returns:
            tuple: (best_vx, best_vy, best_omega, best_trajectory)
        """
        # Calculate dynamic window (now includes Va)
        dw = self.calculate_dynamic_window()
        
        # Initialize best trajectory
        best_cost = float('inf')
        best_vx, best_vy, best_omega = 0.0, 0.0, 0.0
        best_trajectory = None
        
        # Sample velocities within dynamic window
        for vx in np.arange(dw[0], dw[1], self.config.vel_resolution_x):
            for vy in np.arange(dw[2], dw[3], self.config.vel_resolution_y):
                for omega in np.arange(dw[4], dw[5], self.config.vel_resolution_theta):
                    # Evaluate this velocity combination
                    cost, trajectory = self.evaluate_trajectory(vx, vy, omega)
                    
                    # Update best trajectory if this is better
                    if cost < best_cost:
                        best_cost = cost
                        best_vx = vx
                        best_vy = vy
                        best_omega = omega
                        best_trajectory = trajectory
        
        return best_vx, best_vy, best_omega, best_trajectory
    
    def is_goal_reached(self):
        """
        Check if robot has reached the goal within tolerance.
        
        Returns:
            bool: True if goal is reached
        """
        if self.goal_pose is None or self.current_pose is None:
            return False
        
        # Calculate distance to goal
        dx = self.goal_pose[0] - self.current_pose[0]
        dy = self.goal_pose[1] - self.current_pose[1]
        distance = np.hypot(dx, dy)
        
        # Calculate angular difference
        angle_diff = abs(np.arctan2(np.sin(self.goal_pose[2] - self.current_pose[2]),
                                    np.cos(self.goal_pose[2] - self.current_pose[2])))
        
        return (distance < self.config.goal_tolerance_xy and 
                angle_diff < self.config.goal_tolerance_theta)
    
    def publish_cmd_vel(self, vx, vy, omega):
        """
        Publish velocity command.
        
        Args:
            vx, vy: Linear velocities in robot frame [m/s]
            omega: Angular velocity [rad/s]
        """
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.angular.z = omega
        self.cmd_vel_pub.publish(cmd)
    
    def publish_local_path(self, trajectory):
        """
        Publish predicted trajectory for visualization in RViz.
        
        Args:
            trajectory: Predicted trajectory points
        """
        if trajectory is None:
            return
        
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "odom"
        
        for point in trajectory:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0.0
            
            # Convert theta to quaternion
            pose.pose.orientation.z = np.sin(point[2] / 2.0)
            pose.pose.orientation.w = np.cos(point[2] / 2.0)
            
            path_msg.poses.append(pose)
        
        self.local_path_pub.publish(path_msg)
    
    def run(self):
        """
        Main control loop.
        """
        rospy.loginfo("DWA Planner running. Waiting for goal...")
        
        while not rospy.is_shutdown():
            # Check if we have necessary data
            if self.current_pose is None:
                rospy.logwarn_throttle(5, "Waiting for odometry data...")
                self.rate.sleep()
                continue
            
            # If no goal, stop the robot
            if self.goal_pose is None:
                self.publish_cmd_vel(0.0, 0.0, 0.0)
                self.rate.sleep()
                continue
            
            # Check if goal is reached
            if self.is_goal_reached():
                rospy.loginfo("Goal reached!")
                self.publish_cmd_vel(0.0, 0.0, 0.0)
                self.goal_pose = None  # Clear goal
                self.rate.sleep()
                continue
            
            # Run DWA algorithm
            vx, vy, omega, trajectory = self.dwa_control()
            
            # Publish commands
            self.publish_cmd_vel(vx, vy, omega)
            self.publish_local_path(trajectory)
            
            # Log current status
            rospy.loginfo_throttle(1, f"CMD: vx={vx:.2f}, vy={vy:.2f}, omega={omega:.2f}")
            
            self.rate.sleep()
            
        #Parar al terminar 
        self.publish_cmd_vel(0.0, 0.0, 0.0)


if __name__ == '__main__':
    try:
        planner = DWAPlannerMecanum()
        planner.run()
    except rospy.ROSInterruptException:
        pass
