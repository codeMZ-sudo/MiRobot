#!/usr/bin/env python3
import rospy
import math
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class PathFollower:
    def __init__(self):
        rospy.init_node('path_to_goal_node', anonymous=True)

        # Parameters
        self.lookahead_distance = rospy.get_param("~lookahead_distance", 0.8)
        self.update_rate = rospy.get_param("~update_rate", 10.0)
        self.path_topic = rospy.get_param("~path_topic", "/global_path")
        self.goal_topic = rospy.get_param("~goal_topic", "/goal")
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.frame_id = rospy.get_param("~frame_id", "map")

        # Internal state
        self.current_path = []
        self.robot_pose = None
        self.final_goal_yaw = 0.0  # default orientation from user goal
        self.final_goal = None

        # Subscribers
        rospy.Subscriber(self.path_topic, Path, self.path_callback)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.user_goal_callback)  # user-set RViz goal

        # Publisher
        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1)

        rospy.loginfo("[PathFollower] Node initialized. Waiting for /global_path and /goal ...")
        self.loop()

    def user_goal_callback(self, msg: PoseStamped):
        """Store the final goal pose and yaw (sent from RViz)."""
        self.final_goal = (msg.pose.position.x, msg.pose.position.y)
        quat = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        )
        _, _, yaw = euler_from_quaternion(quat)
        self.final_goal_yaw = yaw
        rospy.loginfo(f"[PathFollower] Received user goal with yaw={math.degrees(yaw):.1f}°")

    def odom_callback(self, msg):
        """Store robot's current (x, y)."""
        self.robot_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

    def path_callback(self, msg):
        """Convert nav_msgs/Path to list of (x, y) tuples."""
        self.current_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        rospy.loginfo_throttle(2.0, f"[PathFollower] Received new path with {len(self.current_path)} points.")

    def get_lookahead_point(self):
        """Find a point ahead of robot along path and compute yaw direction."""
        if not self.robot_pose or not self.current_path:
            return None, None

        rx, ry = self.robot_pose
        closest_idx = None
        closest_dist = float('inf')

        # 1. Find nearest path point
        for i, (px, py) in enumerate(self.current_path):
            d = math.hypot(px - rx, py - ry)
            if d < closest_dist:
                closest_dist = d
                closest_idx = i

        if closest_idx is None:
            return None, None

        # 2. Find a lookahead point ahead of robot
        path_len = len(self.current_path)
        target_point = self.current_path[-1]  # default to last point
        target_idx = path_len - 1

        for j in range(closest_idx, path_len):
            px, py = self.current_path[j]
            if math.hypot(px - rx, py - ry) >= self.lookahead_distance:
                target_point = (px, py)
                target_idx = j
                break

        # 3. Compute yaw
        # Case A: Last point → use user-specified goal yaw
        if target_idx >= path_len - 1 and self.final_goal is not None:
            yaw = self.final_goal_yaw
        else:
            # Case B: Intermediate → use direction of local trajectory
            next_idx = min(target_idx + 1, path_len - 1)
            px2, py2 = self.current_path[next_idx]
            yaw = math.atan2(py2 - target_point[1], px2 - target_point[0])

        return target_point, yaw

    def loop(self):
        """Continuously publish lookahead goal with orientation."""
        rate = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown():
            target, yaw = self.get_lookahead_point()
            if target and yaw is not None:
                gx, gy = target
                q = quaternion_from_euler(0, 0, yaw)

                goal_msg = PoseStamped()
                goal_msg.header.stamp = rospy.Time.now()
                goal_msg.header.frame_id = self.frame_id
                goal_msg.pose.position.x = gx
                goal_msg.pose.position.y = gy
                goal_msg.pose.orientation = Quaternion(*q)

                self.goal_pub.publish(goal_msg)
                #rospy.loginfo_throttle(2.0, f"[PathFollower] Goal: ({gx:.2f}, {gy:.2f}), yaw={math.degrees(yaw):.1f}°")
            else:
                rospy.logwarn_throttle(5.0, "[PathFollower] Waiting for valid path, odom, or goal...")

            rate.sleep()


if __name__ == "__main__":
    try:
        PathFollower()
    except rospy.ROSInterruptException:
        pass

