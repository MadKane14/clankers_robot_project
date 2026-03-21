import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import math
import json
import os

class BatterySimulator(Node):
    def __init__(self):
        super().__init__('battery_simulator')

        self.battery_level = 100.0  
        self.consumption_rate = 0.5 
        self.charge_rate = 5.0      
        
        # --- Dynamically Load & Translate Chargers ---
        map_path = os.path.expanduser('~/ros2_ws/src/clankers_robot_project/mission_planner/mission_planner/topological_map.json')
        try:
            with open(map_path, 'r') as f:
                map_data = json.load(f)
                nodes = map_data['nodes']
                
            spawn_x = float(nodes['start'][0])
            spawn_y = float(nodes['start'][1])
            
            self.charging_stations = []
            for name, coords in nodes.items():
                if 'charger' in name:
                    rel_x = float(coords[0]) - spawn_x
                    rel_y = float(coords[1]) - spawn_y
                    self.charging_stations.append((rel_x, rel_y))
                    self.get_logger().info(f"Loaded {name} at relative coords: ({rel_x:.2f}, {rel_y:.2f})")
        except FileNotFoundError:
            self.get_logger().error("Could not find JSON map!")
            self.charging_stations = []

        self.charging_radius = 1.0  
        self.is_charging = False
        self.last_x = None
        self.last_y = None

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.battery_pub = self.create_publisher(Float32, '/robot_battery/status', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("Battery Simulator Node Started at 100%")

    def odom_callback(self, msg):
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        if self.last_x is None or self.last_y is None:
            self.last_x = current_x
            self.last_y = current_y
            return

        self.is_charging = False
        for (cx, cy) in self.charging_stations:
            distance = math.sqrt((current_x - cx)**2 + (current_y - cy)**2)
            if distance <= self.charging_radius:
                self.is_charging = True
                break
            
        if not self.is_charging:
            distance_traveled = math.sqrt((current_x - self.last_x)**2 + (current_y - self.last_y)**2)
            drain_amount = distance_traveled * self.consumption_rate
            self.battery_level -= drain_amount

        self.last_x = current_x
        self.last_y = current_y

        if self.battery_level < 0.0:
            self.battery_level = 0.0

    def timer_callback(self):
        if self.is_charging:
            self.battery_level += self.charge_rate
            if self.battery_level > 100.0:
                self.battery_level = 100.0
            self.get_logger().info(f"Charging... Battery: {self.battery_level:.1f}%")

        msg = Float32()
        msg.data = self.battery_level
        self.battery_pub.publish(msg)

        if self.battery_level < 100.0 and not self.is_charging:
            self.get_logger().warn(f"LOW BATTERY: {self.battery_level:.1f}%")

def main(args=None):
    rclpy.init(args=args)
    node = BatterySimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()