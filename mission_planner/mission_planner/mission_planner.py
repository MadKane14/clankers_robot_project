import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
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

        # --- Subscribers & Action Clients ---
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.battery_sub = self.create_subscription(Float32, '/robot_battery/status', self.battery_callback, 10)
        
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.timer = self.create_timer(2.0, self.planning_loop)

    # --- SENSOR CALLBACKS ---
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def battery_callback(self, msg):
        self.battery_level = msg.data

    # --- MISSION LOGIC ---
    def planning_loop(self):
        if self.robot_is_moving:
            return

        if not self.pending_tasks:
            self.get_logger().info("ALL DELIVERIES COMPLETED! Mission Success.")
            self.timer.cancel()
            return

        # Charging Check
        if self.current_node in self.chargers and self.battery_level < 99.0:
            self.get_logger().info(f"Charging... Battery: {self.battery_level:.1f}%")
            return

        # Simple Nearest Neighbor Logic
        closest_task = min(self.pending_tasks, key=lambda x: self.graph[self.current_node][x])
        cost_to_task = self.graph[self.current_node][closest_task] * self.energy_drain_rate

        closest_charger_from_task = min(self.chargers, key=lambda x: self.graph[closest_task][x])
        cost_to_charger_after = self.graph[closest_task][closest_charger_from_task] * self.energy_drain_rate

        total_required_energy = cost_to_task + cost_to_charger_after

        # Dispatch Decision
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
        
        # Calculate safe parking offset so it doesn't crash into the cylinder
        angle_to_obj = math.atan2(rel_y, rel_x)
        safe_x = rel_x - 0.65 * math.cos(angle_to_obj)
        safe_y = rel_y - 0.65 * math.sin(angle_to_obj)

        self.get_logger().info(f"Dispatching Global Nav2 Goal to {target_node_name} at ({safe_x:.2f}, {safe_y:.2f})")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = safe_x
        goal_msg.pose.pose.position.y = safe_y
        goal_msg.pose.pose.orientation.w = 1.0 
        
        self.send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(
            lambda future, node=target_node_name: self.goal_response_callback(future, node)
        )

    def goal_response_callback(self, future, target_node_name):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 rejected the goal!')
            self.failed_attempts += 1
            self.robot_is_moving = False
            return

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(
            lambda future, node=target_node_name: self.get_result_callback(future, node)
        )

    def get_result_callback(self, future, target_node_name):
        result = future.result().status
        if result == 4: # SUCCEEDED
            self.failed_attempts = 0 
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