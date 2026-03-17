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
        self.get_logger().info("True Plunge Explorer Initialized.")

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.is_navigating = False
        
        self.blacklist = []
        self.failed_deep_edges = []  
        self.permanent_blacklist = [] 
        
        self.current_edge = None
        self.current_target = None
        self.last_cleared_target = None 
        self.is_current_target_deep = True 
        
        self.blacklist_radius = 0.60  
        
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 1)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.nav_start_time = None
        self.goal_handle = None
        
        self.anchor_x = 0.0
        self.anchor_y = 0.0
        self.anchor_time = None
        
        self.watchdog_timer = self.create_timer(0.5, self.watchdog_callback)
        
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
        
        free_space = (grid >= 0) & (grid < 25)
        unknown_space = (grid == -1)

        dilated_unknown = binary_dilation(unknown_space)
        frontiers = free_space & dilated_unknown
        labeled_frontiers, num_features = label(frontiers)

        if num_features == 0:
            self.get_logger().info("MAPPING COMPLETE. NO FRONTIERS FOUND.")
            return

        best_edge = None
        best_deep_target = None
        lowest_cost = float('inf')
        valid_frontiers_count = 0
        is_best_target_deep = True

        for i in range(1, num_features + 1):
            blob_pixels = (labeled_frontiers == i)
            blob_size = np.sum(blob_pixels)

            if blob_size < 15: 
                continue

            center_row, center_col = center_of_mass(blob_pixels)
            edge_x = orig_x + (center_col * res)
            edge_y = orig_y + (center_row * res)

            angle_to_edge = math.atan2(edge_y - self.robot_y, edge_x - self.robot_x)
            
            is_failed_deep = False
            for fx, fy in self.failed_deep_edges:
                if math.sqrt((edge_x - fx)**2 + (edge_y - fy)**2) < 0.5:
                    is_failed_deep = True
                    break

            if is_failed_deep:
                target_x = edge_x - (0.80 * math.cos(angle_to_edge))
                target_y = edge_y - (0.80 * math.sin(angle_to_edge))
                current_is_deep = False
            else:
                max_plunge = 2.0
                step_size = 0.1
                current_plunge = 0.0
                
                target_x = edge_x
                target_y = edge_y
                
                while current_plunge < max_plunge:
                    test_x = edge_x + (current_plunge + step_size) * math.cos(angle_to_edge)
                    test_y = edge_y + (current_plunge + step_size) * math.sin(angle_to_edge)
                    
                    test_c = int((test_x - orig_x) / res)
                    test_r = int((test_y - orig_y) / res)
                    
                    if test_r < 0 or test_r >= height or test_c < 0 or test_c >= width:
                        break
                    if grid[test_r, test_c] > 50:
                        break
                        
                    target_x = test_x
                    target_y = test_y
                    current_plunge += step_size

                if current_plunge < 0.3:
                    continue
                current_is_deep = True

            distance_to_edge = math.sqrt((edge_x - self.robot_x)**2 + (edge_y - self.robot_y)**2)
            
            # --- THE BLIND SPOT FIX ---
            # Increased from 0.20 to 0.55. If the robot is sitting right on top of a frontier, 
            # it mathematically ignores it. This prevents the instant-victory loop!
            if distance_to_edge < 0.55: 
                continue
                
            is_blacklisted = False
            for bx, by in self.blacklist + self.permanent_blacklist:
                if math.sqrt((edge_x - bx)**2 + (edge_y - by)**2) < self.blacklist_radius: 
                    is_blacklisted = True
                    break
            
            if is_blacklisted:
                continue

            valid_frontiers_count += 1
            cost = distance_to_edge

            if self.last_cleared_target:
                dist_to_last_work_zone = math.sqrt((edge_x - self.last_cleared_target[0])**2 + (edge_y - self.last_cleared_target[1])**2)
                cost += (dist_to_last_work_zone * 2.0) 

            if cost < lowest_cost:
                lowest_cost = cost
                best_edge = (edge_x, edge_y)
                best_deep_target = (target_x, target_y)
                is_best_target_deep = current_is_deep

        if best_deep_target and best_edge:
            self.current_edge = best_edge
            self.current_target = best_deep_target
            self.is_current_target_deep = is_best_target_deep
            
            self.get_logger().info(f"Analyzed {num_features} blobs. Found {valid_frontiers_count} valid frontiers.")
            
            if is_best_target_deep:
                actual_depth = math.sqrt((best_deep_target[0] - best_edge[0])**2 + (best_deep_target[1] - best_edge[1])**2)
                self.get_logger().info(f"Edge at ({best_edge[0]:.2f}, {best_edge[1]:.2f}). Plunging {actual_depth:.2f}m deep.")
            else:
                self.get_logger().warn(f"Edge at ({best_edge[0]:.2f}, {best_edge[1]:.2f}) failed. BACKING DOWN 0.80m.")
                
            self.send_goal(best_deep_target[0], best_deep_target[1])
        else:
            if len(self.blacklist) > 0 or len(self.failed_deep_edges) > 0:
                self.get_logger().warn("Frontiers exhausted! Declaring Amnesty (Clearing temporary memory)...")
                self.blacklist.clear()
                self.failed_deep_edges.clear()
            else:
                self.get_logger().warn("Waiting for map updates or MAPPING COMPLETE.")

    def send_goal(self, target_x, target_y):
        self.is_navigating = True
        self.nav_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(target_x)
        goal_msg.pose.pose.position.y = float(target_y)
        goal_msg.pose.pose.orientation.w = 1.0

        self.send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result() 
        
        if not self.goal_handle.accepted:
            if self.is_current_target_deep:
                self.get_logger().warn('Nav2 rejected DEEP target! Adding to Strike 1 memory.')
                self.failed_deep_edges.append(self.current_edge)
            else:
                self.get_logger().error('Nav2 rejected BACKED-DOWN target! Blacklisting PERMANENTLY.')
                self.permanent_blacklist.append(self.current_edge)
                
            self.is_navigating = False
            return

        self.nav_start_time = self.get_clock().now() 
        self.anchor_x = self.robot_x
        self.anchor_y = self.robot_y
        self.anchor_time = self.nav_start_time
        self.cancel_reason = None
        
        self.get_result_future = self.goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().status
        
        if result == 4 or (result == 5 and self.cancel_reason == 'VICTORY'):
            self.get_logger().info("Target successfully cleared.")
            if self.current_edge:
                self.blacklist.append(self.current_edge)
                self.last_cleared_target = self.current_edge
        else:
            self.get_logger().warn(f"Failed to reach target (Nav2 Status: {result}). Reason: {self.cancel_reason}")
            if self.current_edge:
                if self.is_current_target_deep:
                    self.get_logger().info("Deep plunge failed. Will attempt to back down next loop.")
                    self.failed_deep_edges.append(self.current_edge)
                else:
                    self.get_logger().error("Backed-down target also failed. Blacklisting PERMANENTLY.")
                    self.permanent_blacklist.append(self.current_edge) 
        
        self.current_edge = None
        self.current_target = None
        self.is_navigating = False
        self.nav_start_time = None
        self.anchor_time = None
    
    def watchdog_callback(self):
        if not (self.is_navigating and self.nav_start_time and self.goal_handle and self.current_target):
            return

        current_time = self.get_clock().now()
        
        # --- THE TRUE PLUNGE FIX ---
        # We check the distance to the TARGET, not the edge!
        dist_to_target = math.sqrt((self.current_target[0] - self.robot_x)**2 + (self.current_target[1] - self.robot_y)**2)

        # Because the target is safely ray-casted ahead of obstacles, the robot can successfully
        # drive up to it. If it gets within 0.4m of the final destination, we guarantee it crossed the edge!
        if dist_to_target < 0.4:
            self.get_logger().info(f"Deep target reached. Canceling goal...")
            self.cancel_reason = 'VICTORY'
            self.goal_handle.cancel_goal_async()
            return

        if self.anchor_time:
            time_since_anchor = (current_time - self.anchor_time).nanoseconds / 1e9
            
            if time_since_anchor > 15.0:
                dist_from_anchor = math.sqrt((self.robot_x - self.anchor_x)**2 + (self.robot_y - self.anchor_y)**2)
                
                if dist_from_anchor < 0.02:
                    self.get_logger().error(f"Robot physically stuck. Aborting goal...")
                    self.cancel_reason = 'STUCK'
                    self.goal_handle.cancel_goal_async()
                    return
                else:
                    self.anchor_x = self.robot_x
                    self.anchor_y = self.robot_y
                    self.anchor_time = current_time

        total_elapsed_time = (current_time - self.nav_start_time).nanoseconds / 1e9
        if total_elapsed_time > 180.0:
            self.get_logger().error(f"Goal exceeded absolute maximum time ({total_elapsed_time:.1f}s). Aborting...")
            self.cancel_reason = 'MAX_TIMEOUT'
            self.goal_handle.cancel_goal_async()
            return

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()