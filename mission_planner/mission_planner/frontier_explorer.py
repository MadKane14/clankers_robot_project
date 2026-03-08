import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
import numpy as np
from scipy.ndimage import binary_dilation, label, center_of_mass
import math

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        self.get_logger().info("Frontier Exploration Node Initialized. Waiting for map...")

        # --- State Variables ---
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.is_navigating = False
        
        # --- NEW: Memory & Blacklist ---
        self.blacklist = []
        self.current_target = None
        
        # --- Subscribers & Action Client ---
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 1)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.nav_start_time = None
        self.goal_handle = None
        
        # --- NEW: Watchdog Timer (Checks every 1 second) ---
        self.watchdog_timer = self.create_timer(1.0, self.watchdog_callback)
        
    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def map_callback(self, msg):
        if self.is_navigating:
            return  

        info = msg.info
        width = info.width
        height = info.height
        res = info.resolution
        orig_x = info.origin.position.x
        orig_y = info.origin.position.y

        grid = np.array(msg.data).reshape((height, width))
        
        # --- FIX 1: Tolerate SLAM noise (0 to 24 is free space) ---
        free_space = (grid >= 0) & (grid < 25)
        unknown_space = (grid == -1)

        dilated_unknown = binary_dilation(unknown_space)
        frontiers = free_space & dilated_unknown

        labeled_frontiers, num_features = label(frontiers)

        if num_features == 0:
            self.get_logger().info("NO FRONTIERS FOUND! MAPPING COMPLETE.")
            self.get_logger().info("ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/clankers_robot_project/my_robot_nav/maps/custom_world_map")
            return

        best_frontier = None
        lowest_cost = float('inf')
        valid_frontiers_count = 0

        for i in range(1, num_features + 1):
            blob_pixels = (labeled_frontiers == i)
            blob_size = np.sum(blob_pixels)

            if blob_size < 4: 
                continue

            center_row, center_col = center_of_mass(blob_pixels)
            target_x = orig_x + (center_col * res)
            target_y = orig_y + (center_row * res)

            distance = math.sqrt((target_x - self.robot_x)**2 + (target_y - self.robot_y)**2)
            
            # --- AGGRESSIVE PROXIMITY FILTER ---
            # Force the robot to take bigger steps. Ignore anything closer than 1.2 meters.
            if distance < 1.2:
                continue
                
            is_blacklisted = False
            for bx, by in self.blacklist:
                # --- FIX: Expanded Blacklist Radius to 0.75m ---
                if math.sqrt((target_x - bx)**2 + (target_y - by)**2) < 0.75: 
                    is_blacklisted = True
                    break
            
            if is_blacklisted:
                continue

            valid_frontiers_count += 1

            # --- INFORMATION GAIN HEURISTIC ---
            # Subtract a fraction of the blob size from the distance. 
            # Massive frontiers will now have a lower "cost" and outrank closer, smaller ones.
            cost = distance - (blob_size * 0.05)

            if cost < lowest_cost:
                lowest_cost = cost
                best_frontier = (target_x, target_y)

        if best_frontier:
            # --- THE FIX: Safe Parking Vector Pullback ---
            # Calculate the angle from the robot to the frontier
            angle = math.atan2(best_frontier[1] - self.robot_y, best_frontier[0] - self.robot_x)
            
            # Pull the target back by 0.55 meters into guaranteed safe space
            safe_x = best_frontier[0] - 0.55 * math.cos(angle)
            safe_y = best_frontier[1] - 0.55 * math.sin(angle)
            
            # --- FIX: Store actual frontier coordinate for the blacklist memory ---
            self.current_target = best_frontier
            
            self.get_logger().info(f"Analyzed {num_features} blobs. Found {valid_frontiers_count} valid frontiers.")
            self.get_logger().info(f"Targeting safe offset at ({safe_x:.2f}, {safe_y:.2f}) instead of exact edge.")
            self.send_goal(safe_x, safe_y)
        else:
            self.get_logger().warn(f"Analyzed {num_features} blobs, but 0 passed the size/blacklist filters. Waiting for map to update...")

    def send_goal(self, target_x, target_y):
        self.is_navigating = True
        self.nav_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = target_x
        goal_msg.pose.pose.position.y = target_y
        goal_msg.pose.pose.orientation.w = 1.0

        self.send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result() # Save the handle!
        
        # --- THE FIX: Added 'self.' before goal_handle ---
        if not self.goal_handle.accepted:
            self.get_logger().error('Nav2 rejected the frontier goal!')
            self.blacklist.append(self.current_target) # Blacklist it so we don't try again
            self.is_navigating = False
            return

        # Start the clock the moment the goal is accepted
        self.nav_start_time = self.get_clock().now() 
        
        self.get_result_future = self.goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().status
        
        # Handle the different ways a goal can finish
        if result == 4: # SUCCEEDED (Nav2 actually touched the pixel)
            self.get_logger().info("Reached frontier exactly.")
        elif result == 5 and self.cancel_reason == 'VICTORY': # CANCELED by our distance checker
            self.get_logger().info("Frontier cleared by proximity. Moving on.")
        else: # ABORTED or CANCELED by timeout
            self.get_logger().warn(f"Failed to reach frontier (Status: {result}). Reason: {self.cancel_reason}")
        
        # Always blacklist the target so we don't repeat it, then unlock the planner
        if self.current_target:
            self.blacklist.append(self.current_target)
            self.current_target = None
            
        self.is_navigating = False
        self.nav_start_time = None
        self.cancel_reason = None
    
    def watchdog_callback(self):
        # Ensure we are actively navigating and have a target
        if not (self.is_navigating and self.nav_start_time and self.goal_handle and self.current_target):
            return

        elapsed_time = (self.get_clock().now() - self.nav_start_time).nanoseconds / 1e9
        
        # Calculate current physical distance to the goal
        dist_to_target = math.sqrt((self.current_target[0] - self.robot_x)**2 + (self.current_target[1] - self.robot_y)**2)

        # 1. VISUAL RANGE VICTORY (The Lenient Success)
        if dist_to_target < 3.0:
            self.get_logger().info(f"Visual range reached ({dist_to_target:.1f}m). LiDAR has scanned the area!")
            self.cancel_reason = 'VICTORY'
            self.goal_handle.cancel_goal_async() # Manually stop Nav2
            return

        # 2. GENUINE TIMEOUT (The Hard Stuck Fail-safe)
        if elapsed_time > 67.0:
            self.get_logger().error(f"Robot physically stuck for {elapsed_time:.1f}s. Aborting goal...")
            self.cancel_reason = 'TIMEOUT'
            self.goal_handle.cancel_goal_async()
            return

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()