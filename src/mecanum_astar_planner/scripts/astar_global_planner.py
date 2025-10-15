#!/usr/bin/env python3
"""
ROS-adapted A* Global Planner
Loads PGM maps and publishes paths
Based on zhm-real/PathPlanning implementation
"""

import rospy
import numpy as np
import math
import heapq
from collections import defaultdict
from PIL import Image
import yaml

from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


class ROSAStarPlanner:
    """
    A* Global Planner adapted for ROS with PGM map support
    """
    
    def __init__(self):
        rospy.init_node('astar_global_planner', anonymous=False)
        
        # Parameters
        self.map_file = rospy.get_param('~map_file', 'map.yaml')
        self.heuristic_type = rospy.get_param('~heuristic_type', 'euclidean')
        self.inflation_radius = rospy.get_param('~inflation_radius', 0.2)  # meters
        
        # Map data
        self.occupancy_grid = None
        self.resolution = None
        self.origin = None
        self.width = None
        self.height = None
        self.obs_set = set()
        
        # Motion primitives (8-directional for mecanum)
        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                       (1, 0), (1, -1), (0, -1), (-1, -1)]
        
        # Publishers
        self.path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
        self.marker_pub = rospy.Publisher('/astar_visualization', MarkerArray, queue_size=1)
        self.marker_ocp_pub = rospy.Publisher('/ocp_visualization', MarkerArray, queue_size=1)
        
        # Load map
        self.load_map(self.map_file)
        
        rospy.loginfo("A* Global Planner initialized and ready!")
        rospy.loginfo(f"Map size: {self.width}x{self.height}, Resolution: {self.resolution}m")
        
        # Subscribers
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        
        # Current robot pose
        self.robot_pose = None
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
    
    def load_map(self, yaml_file):
        """
        Load PGM map from YAML file
        
        Args:
            yaml_file: Path to map.yaml file
        """
        try:
            # Load YAML
            with open(yaml_file, 'r') as f:
                map_metadata = yaml.safe_load(f)
            
            # Extract metadata
            self.resolution = map_metadata['resolution']
            self.origin = map_metadata['origin']  # [x, y, theta]
            occupied_thresh = map_metadata.get('occupied_thresh', 0.65)
            free_thresh = map_metadata.get('free_thresh', 0.196)
            
            # Load PGM image
            import os
            yaml_dir = os.path.dirname(yaml_file)
            pgm_file = os.path.join(yaml_dir, map_metadata['image'])
            
            img = Image.open(pgm_file).convert('L')  # Convert to grayscale
            img_array = np.array(img)
            
            self.height, self.width = img_array.shape
            
            # Convert to occupancy grid (0-100 scale)
            # PGM: 255 = free, 0 = occupied
            # Occupancy: 0 = free, 100 = occupied
            occupancy_data = np.zeros((self.height, self.width), dtype=np.int8)
            
            # Threshold values
            occupied_threshold = int(255 * (1 - occupied_thresh))
            free_threshold = int(255 * (1 - free_thresh))
            
            # Classify cells
            occupancy_data[img_array <= occupied_threshold] = 100  # Occupied
            occupancy_data[img_array >= free_threshold] = 0        # Free
            occupancy_data[(img_array > occupied_threshold) & 
                          (img_array < free_threshold)] = -1     # Unknown
            
            
            self.occupancy_grid = occupancy_data
            
            # Build obstacle set with inflation
            self.build_obstacle_set()
            
            # Publish map for visualization
            self.publish_map()
            
            rospy.loginfo(f"Map loaded successfully from {pgm_file}")
            rospy.loginfo(f"Found {len(self.obs_set)} occupied cells")
            
        except Exception as e:
            rospy.logerr(f"Failed to load map: {e}")
            raise
    
    def build_obstacle_set(self):
        """
        Build set of obstacle cells with inflation
        IMPORTANT: Create new set to avoid RuntimeError during iteration
        """
        # First, find all originally occupied cells
        occupied = np.argwhere(self.occupancy_grid >= 50)  # Threshold for occupied
        
        # Now create inflated obstacle set
        inflated_obs = set()
        inflation_cells = int(self.inflation_radius / self.resolution)
        
        for y, x in occupied:
            # Add cell and inflated neighbors
            for dx in range(-inflation_cells, inflation_cells + 1):
                for dy in range(-inflation_cells, inflation_cells + 1):
                    ix, iy = x + dx, self.height - y + dy
                    if 0 <= ix < self.width and 0 <= iy < self.height:
                        # Use circular inflation
                        if dx*dx + dy*dy <= inflation_cells*inflation_cells:
                            inflated_obs.add((ix, iy))
        
        # Assign the complete inflated set at once
        self.obs_set = inflated_obs
        
        self.visualize_ocuppancy_grid()
        
        rospy.loginfo(f"After inflation by {self.inflation_radius}m ({inflation_cells} cells): {len(self.obs_set)}")
    
    def world_to_grid(self, x, y):
        """
        Convert world coordinates to grid indices
        
        Args:
            x, y: World coordinates (meters)
        Returns:
            (grid_x, grid_y): Grid indices
        """
        grid_x = int((x - self.origin[0]) / self.resolution)
        grid_y = int((y - self.origin[1]) / self.resolution)
        return (grid_x, grid_y)
    
    def grid_to_world(self, grid_x, grid_y):
        """
        Convert grid indices to world coordinates
        
        Args:
            grid_x, grid_y: Grid indices
        Returns:
            (x, y): World coordinates (meters)
        """
        x = grid_x * self.resolution + self.origin[0]
        y = grid_y * self.resolution + self.origin[1]
        return (x, y)
    
    def odom_callback(self, msg):
        """
        Update current robot pose and velocity from odometry.
        """
        # Extract position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        self.robot_pose = (x,y)
        
    def goal_callback(self, msg):
        """
        Handle goal from RViz '2D Nav Goal'
        
        Args:
            msg: PoseStamped message
        """
        if self.robot_pose is None:
            rospy.logwarn("Robot pose not available yet, cannot plan path")
            return
        
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        
        rospy.loginfo(f"Received goal: ({goal_x:.2f}, {goal_y:.2f})")
        
        # Plan path
        self.plan_and_publish(self.robot_pose, (goal_x, goal_y))
    
    def plan_and_publish(self, start_world, goal_world):
        """
        Plan path from start to goal and publish
        
        Args:
            start_world: (x, y) in world coordinates
            goal_world: (x, y) in world coordinates
        """
        # Convert to grid coordinates
        start_grid = self.world_to_grid(*start_world)
        goal_grid = self.world_to_grid(*goal_world)
        
        rospy.loginfo(f"Planning from {start_grid} to {goal_grid} (grid)")
        
        # Validate
        if not self.is_valid_cell(start_grid[0], start_grid[1]):
            rospy.logerr("Start position is in obstacle!")
            return
        
        if not self.is_valid_cell(goal_grid[0], goal_grid[1]):
            rospy.logerr("Goal position is in obstacle!")
            return
        
        # Run A*
        path_grid, visited = self.astar_search(start_grid, goal_grid)
        
        if path_grid is None:
            rospy.logerr("No path found!")
            return
        
        # Convert path to world coordinates
        path_world = [self.grid_to_world(x, y) for x, y in path_grid]
        
        # Publish path
        self.publish_path(path_world)
        
        self.visualize_ocuppancy_grid()
        
        # Visualize
        #self.visualize_search(visited, path_grid)
        
        
        rospy.loginfo(f"Path found with {len(path_world)} waypoints")
    
    def astar_search(self, start, goal):
        """
        Improved A*:
          - CLOSED is a set (fast membership).
          - Skip neighbors already in CLOSED.
          - Ignore stale heap entries when popped.
        Returns:
          path (list) or None, visited (list)
        """
        open_heap = []
        closed_set = set()
        parent = {start: None}
        g = defaultdict(lambda: float('inf'))
        g[start] = 0

        # push start with f = g + h
        heapq.heappush(open_heap, (g[start] + self.heuristic(start, goal), start))

        visited_order = []  # for visualization; will contain nodes in the order they were popped

        while open_heap:
            f, current = heapq.heappop(open_heap)

            # If we already expanded this node, skip (stale heap entry)
            if current in closed_set:
                continue

            # mark visited
            closed_set.add(current)
            visited_order.append(current)

            if current == goal:
                # reconstruct path
                path = []
                node = goal
                while node is not None:
                    path.append(node)
                    node = parent[node]
                path.reverse()
                return path, visited_order

            for neighbor in self.get_neighbors(current):
                if neighbor in closed_set:
                    # if heuristic is consistent we can skip closed neighbors
                    continue

                tentative_g = g[current] + self.motion_cost(current, neighbor)
                if tentative_g < g[neighbor]:
                    g[neighbor] = tentative_g
                    parent[neighbor] = current
                    heapq.heappush(open_heap, (tentative_g + self.heuristic(neighbor, goal), neighbor))

        # open empty, no path
        return None, visited_order

    
    def get_neighbors(self, cell):
        """
        Get valid neighbors of a cell
        
        Args:
            cell: (x, y) grid coordinates
        Returns:
            List of valid neighbor coordinates
        """
        neighbors = []
        for dx, dy in self.motions:
            nx, ny = cell[0] + dx, cell[1] + dy
            if self.is_valid_cell(nx, ny):
                neighbors.append((nx, ny))
        return neighbors
    
    def is_valid_cell(self, x, y):
        """
        Check if cell is valid (in bounds and not obstacle)
        
        Args:
            x, y: Grid coordinates
        Returns:
            True if valid, False otherwise
        """
        if x < 0 or x >= self.width or y < 0 or y >= self.height:
            return False
        return (x, y) not in self.obs_set
    
    def motion_cost(self, cell1, cell2):
        """
        Cost of moving from cell1 to cell2
        
        Args:
            cell1, cell2: Grid coordinates
        Returns:
            Cost (Euclidean distance)
        """
        dx = cell2[0] - cell1[0]
        dy = cell2[1] - cell1[1]
        return math.sqrt(dx*dx + dy*dy)
    
    def heuristic(self, cell, goal):
        """
        Heuristic function (h-value)
        
        Args:
            cell: Current cell
            goal: Goal cell
        Returns:
            Heuristic value
        """
        if self.heuristic_type == "manhattan":
            return abs(goal[0] - cell[0]) + abs(goal[1] - cell[1])
        else:  # euclidean
            return math.sqrt((goal[0] - cell[0])**2 + (goal[1] - cell[1])**2)
    
    def f_value(self, cell, goal, g):
        """
        f = g + h
        
        Args:
            cell: Current cell
            goal: Goal cell
            g: Dictionary of g-values
        Returns:
            f-value
        """
        return g[cell] + self.heuristic(cell, goal)
    
    def publish_path(self, path_world):
        """
        Publish path as nav_msgs/Path
        
        Args:
            path_world: List of (x, y) world coordinates
        """
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"
        
        for x, y in path_world:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
    
    def publish_map(self):
        """Publish occupancy grid for visualization"""
        # This is optional - map_server usually handles this
        pass
    
    def visualize_ocuppancy_grid(self):
        marker_array = MarkerArray()
        #OCP        
        # Create marker for inflated cells
        inflated_marker = Marker()
        inflated_marker.header.frame_id = "map"
        inflated_marker.header.stamp = rospy.Time.now()
        inflated_marker.ns = "inflated_cells"
        inflated_marker.id = 0
        inflated_marker.type = Marker.CUBE_LIST
        inflated_marker.action = Marker.ADD
        inflated_marker.scale.x = self.resolution
        inflated_marker.scale.y = self.resolution
        inflated_marker.scale.z = 0.005
        inflated_marker.pose.orientation.x = 0.0;
        inflated_marker.pose.orientation.y = 0.0;
        inflated_marker.pose.orientation.z = 0.0;
        inflated_marker.pose.orientation.w = 1.0;
        inflated_marker.color = ColorRGBA(1.0, 0.0, 0.0, 0.5)  # Orange, transparent
        
        # Convert sets to lists to avoid RuntimeError during iteration
        obs_set_list = list(self.obs_set)
        
        # Add inflated cells (excluding originally occupied)
        for x, y in obs_set_list:
            world_x, world_y = self.grid_to_world(x, y)
            p = Point()
            p.x, p.y, p.z = world_x, world_y, 0.0
            inflated_marker.points.append(p)
            
        marker_array.markers.append(inflated_marker)
        self.marker_ocp_pub.publish(marker_array)
    
    def visualize_search(self, visited, path_grid):
        """
        Visualize visited nodes and path
        
        Args:
            visited: List of visited cells
            path_grid: List of path cells
        """
        marker_array = MarkerArray()
        
        # Visited cells marker
        visited_marker = Marker()
        visited_marker.header.frame_id = "map"
        visited_marker.header.stamp = rospy.Time.now()
        visited_marker.ns = "visited"
        visited_marker.id = 1
        visited_marker.type = Marker.POINTS
        visited_marker.action = Marker.ADD
        visited_marker.scale.x = self.resolution
        visited_marker.scale.y = self.resolution
        visited_marker.pose.orientation.x = 0.0;
        visited_marker.pose.orientation.y = 0.0;
        visited_marker.pose.orientation.z = 0.0;
        visited_marker.pose.orientation.w = 1.0
        visited_marker.color = ColorRGBA(0.5, 0.5, 0.5, 0.5)  # Gray
        
        for cell in visited:
            x, y = self.grid_to_world(*cell)
            p = Point()
            p.x, p.y, p.z = x, y, 0.0
            visited_marker.points.append(p)
        
        marker_array.markers.append(visited_marker)
        
        # Path marker
        path_marker = Marker()
        path_marker.header.frame_id = "map"
        path_marker.header.stamp = rospy.Time.now()
        path_marker.ns = "path"
        path_marker.id = 1
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.05  # Line width
        path_marker.color = ColorRGBA(0.0, 0.0, 1.0, 0.8)  # Red
        
        for cell in path_grid:
            x, y = self.grid_to_world(*cell)
            p = Point()
            p.x, p.y, p.z = x, y, 0.05
            path_marker.points.append(p)
        
        marker_array.markers.append(path_marker)
        
        self.marker_pub.publish(marker_array)
        
    
    def run(self):
        """Main loop"""
        rospy.spin()


if __name__ == '__main__':
    try:
        planner = ROSAStarPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass
