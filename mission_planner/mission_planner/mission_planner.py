import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry, OccupancyGrid
from nav2_msgs.action import NavigateToPose
import json
import os
import math

class MissionPlannerNode(Node):
    def __init__(self):
        super().__init__('mission_planner_node')
        self.get_logger().info("Mission Planner Initialized. Loading Topological Map...")

        map_path = os.path.expanduser('~/ros2_ws/src/clankers_robot_project/mission_planner/mission_planner/topological_map.json')
        
        try:
            with open(map_path, 'r') as f:
                map_data = json.load(f)
                self.nodes = map_data['nodes']
                self.graph = map_data['graph']
        except FileNotFoundError:
            self.get_logger().error("Could not find topological_map.json!")
            raise SystemExit

        self.spawn_x = float(self.nodes['start'][0])
        self.spawn_y = float(self.nodes['start'][1])

        self.chargers = [name for name in self.nodes.keys() if 'charger' in name]
        self.pending_tasks = [name for name in self.nodes.keys() if 'delivery' in name]
        
        self.current_node = 'start'
        self.robot_is_moving = False
        self.failed_attempts = 0
        
        self.battery_level = 100.0  
        self.energy_drain_rate = 0.5 
        
        self.current_x = 0.0
        self.current_y = 0.0
        
        # --- NEW: Costmap Matrix Variable ---
        self.latest_costmap = None

        # --- Subscribers & Action Clients ---
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.battery_sub = self.create_subscription(Float32, '/robot_battery/status', self.battery_callback, 10)
        
        # --- NEW: Costmap Subscriber ---
        self.costmap_sub = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.costmap_callback, 10)
        
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.timer = self.create_timer(2.0, self.planning_loop)

    # --- SENSOR CALLBACKS ---
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def battery_callback(self, msg):
        self.battery_level = msg.data
        
    def costmap_callback(self, msg):
        self.latest_costmap = msg

    # --- MATH & MATRIX HELPERS ---
    def get_costmap_value(self, x, y):
        """Translates a physical Gazebo (x,y) into a 1D array index."""
        if self.latest_costmap is None:
            return -1  # Map hasn't loaded yet
        
        info = self.latest_costmap.info
        res = info.resolution
        orig_x = info.origin.position.x
        orig_y = info.origin.position.y
        width = info.width
        height = info.height

        col = int(math.floor((x - orig_x) / res))
        row = int(math.floor((y - orig_y) / res))

        if col < 0 or col >= width or row < 0 or row >= height:
            return -1 # Out of bounds
            
        index = row * width + col
        return self.latest_costmap.data[index]

    def is_path_clear(self, start_x, start_y, target_x, target_y):
        """Bresenham-style Raycaster. Checks if line hits lethal obstacles."""
        steps = 25
        for i in range(steps + 1):
            t = i / float(steps)
            sample_x = start_x + t * (target_x - start_x)
            sample_y = start_y + t * (target_y - start_y)
            
            cost = self.get_costmap_value(sample_x, sample_y)
            
            # 100 is lethal, 0 is free, -1 is unknown fog of war.
            # We must ALLOW -1 so the robot can explore unmapped areas.
            if cost >= 60: 
                return False  # Path is physically blocked by a wall
                
        return True

    def calculate_costmap_aware_waypoint(self, curr_x, curr_y, target_x, target_y, step_dist):
        """Sweeps radially to find the nearest safe route around obstacles."""
        target_angle = math.atan2(target_y - curr_y, target_x - curr_x)
        
        # 1. Try Direct Vector (Greedy approach)
        greedy_x = curr_x + step_dist * math.cos(target_angle)
        greedy_y = curr_y + step_dist * math.sin(target_angle)
        
        if self.is_path_clear(curr_x, curr_y, greedy_x, greedy_y):
            self.failed_attempts = 0 # Reset fails if we find a clear direct path
            return greedy_x, greedy_y
            
        # 2. Path Blocked -> Radial Sweep
        self.get_logger().warn("Direct path blocked! Commencing Radial Sweep...")
        angle_step = math.radians(15)
        evade_step = step_dist * 0.75  # Shorter steps for tight corners
        
        for i in range(1, 13): # Sweep up to 180 degrees (12 * 15)
            for sign in [1, -1]:
                test_angle = target_angle + (sign * i * angle_step)
                evade_x = curr_x + evade_step * math.cos(test_angle)
                evade_y = curr_y + evade_step * math.sin(test_angle)
                
                if self.is_path_clear(curr_x, curr_y, evade_x, evade_y):
                    self.get_logger().info(f"Safe gap found at {math.degrees(sign * i * angle_step):.1f} degrees offset.")
                    return evade_x, evade_y
                    
        # 3. Total Entrapment Fail-safe
        self.get_logger().error("Robot is completely surrounded! Executing emergency backup.")
        self.failed_attempts += 1
        return curr_x - 1.0 * math.cos(target_angle), curr_y - 1.0 * math.sin(target_angle)

    # --- MISSION LOGIC ---
    def planning_loop(self):
        if self.robot_is_moving:
            return

        if not self.pending_tasks:
            self.get_logger().info("ALL DELIVERIES COMPLETED! Mission Success.")
            self.timer.cancel()
            return

        if self.current_node in self.chargers and self.battery_level < 99.0:
            self.get_logger().info(f"Charging... Battery: {self.battery_level:.1f}%")
            return

        closest_task = min(self.pending_tasks, key=lambda x: self.graph[self.current_node][x])
        cost_to_task = self.graph[self.current_node][closest_task] * self.energy_drain_rate

        closest_charger_from_task = min(self.chargers, key=lambda x: self.graph[closest_task][x])
        cost_to_charger_after = self.graph[closest_task][closest_charger_from_task] * self.energy_drain_rate

        total_required_energy = cost_to_task + cost_to_charger_after

        if self.battery_level >= (total_required_energy + 5.0):
            if self.failed_attempts == 0:
                self.get_logger().info(f"[DECISION] Targeting: {closest_task}.")
            self.send_nav_goal(closest_task)
        else:
            closest_charger = min(self.chargers, key=lambda x: self.graph[self.current_node][x])
            if self.failed_attempts == 0:
                self.get_logger().warn(f"[DECISION] Low Energy! Diverting to {closest_charger}.")
            self.send_nav_goal(closest_charger)

    def send_nav_goal(self, target_node_name):
        self.robot_is_moving = True
        self.nav_client.wait_for_server()

        raw_x = float(self.nodes[target_node_name][0])
        raw_y = float(self.nodes[target_node_name][1])
        
        rel_x = raw_x - self.spawn_x
        rel_y = raw_y - self.spawn_y
        
        # Calculate safe parking offset
        angle_to_obj = math.atan2(rel_y, rel_x)
        safe_x = rel_x - 0.65 * math.cos(angle_to_obj)
        safe_y = rel_y - 0.65 * math.sin(angle_to_obj)

        dist_to_safe_target = math.sqrt((safe_x - self.current_x)**2 + (safe_y - self.current_y)**2)
        step_distance = 2.5
        
        # --- UPGRADED WAYPOINT LOGIC ---
        if dist_to_safe_target > step_distance:
            target_x, target_y = self.calculate_costmap_aware_waypoint(
                self.current_x, self.current_y, safe_x, safe_y, step_distance
            )
            is_waypoint = True
            if self.failed_attempts == 0:
                self.get_logger().info(f"Target outside known map! Setting Waypoint at ({target_x:.2f}, {target_y:.2f})")
        else:
            target_x = safe_x
            target_y = safe_y
            is_waypoint = False
            self.get_logger().info(f"Target in range! Final approach to ({target_x:.2f}, {target_y:.2f})")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = target_x
        goal_msg.pose.pose.position.y = target_y
        goal_msg.pose.pose.orientation.w = 1.0 
        
        self.send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(
            lambda future, node=target_node_name, wp=is_waypoint: self.goal_response_callback(future, node, wp)
        )

    def goal_response_callback(self, future, target_node_name, is_waypoint):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 rejected the goal!')
            self.failed_attempts += 1
            self.robot_is_moving = False
            return

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(
            lambda future, node=target_node_name, wp=is_waypoint: self.get_result_callback(future, node, wp)
        )

    def get_result_callback(self, future, target_node_name, is_waypoint):
        result = future.result().status
        if result == 4: # SUCCEEDED
            self.failed_attempts = 0 # Reset the stuck tracker!
            if is_waypoint:
                self.get_logger().info(f"Reached Waypoint. SLAM map expanded. Recalculating...")
            else:
                self.get_logger().info(f"ARRIVED SAFELY AT: {target_node_name}")
                self.current_node = target_node_name
                if target_node_name in self.pending_tasks:
                    self.pending_tasks.remove(target_node_name)
        else:
            self.failed_attempts += 1
            self.get_logger().warn(f"Goal failed (Status: {result})")
        
        self.robot_is_moving = False

def main(args=None):
    rclpy.init(args=args)
    node = MissionPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()