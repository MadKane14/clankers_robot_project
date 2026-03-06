#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import math

class BatterySimulator(Node):
    def __init__(self):
        super().__init__('battery_simulator')

        # --- Battery Parameters ---
        self.battery_level = 100.0  # Starts at 100%
        self.consumption_rate = 1.0 # Consumes 1% battery per 1 meter traveled
        self.charge_rate = 5.0      # Recharges 5% battery per 1 second
        
        # --- Service Station Parameters ---
        self.station_x = 0.0
        self.station_y = 0.0
        self.charging_radius = 0.5  # Must be within 0.5m of (0,0) to charge
        self.is_charging = False

        # --- Tracking Variables ---
        self.last_x = None
        self.last_y = None

        # --- Publishers and Subscribers ---
        # Subscribe to odometry to track movement distance
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # Publish the current battery percentage
        self.battery_pub = self.create_publisher(Float32, '/robot_battery/status', 10)

        # Timer to handle continuous charging and publishing (1 Hz / 1 second loops)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info("Battery Simulator Node Started at 100%")

    def odom_callback(self, msg):
        # Extract current position
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        # Initialize last position on first callback
        if self.last_x is None or self.last_y is None:
            self.last_x = current_x
            self.last_y = current_y
            return

        # Calculate distance to the Service Station
        distance_to_station = math.sqrt((current_x - self.station_x)**2 + (current_y - self.station_y)**2)
        
        # Check if we are parked at the station
        if distance_to_station <= self.charging_radius:
            self.is_charging = True
        else:
            self.is_charging = False
            
            # Calculate distance traveled since last odometry message
            distance_traveled = math.sqrt((current_x - self.last_x)**2 + (current_y - self.last_y)**2)
            
            # Drain battery proportional to distance covered
            drain_amount = distance_traveled * self.consumption_rate
            self.battery_level -= drain_amount

        # Update last known position
        self.last_x = current_x
        self.last_y = current_y

        # Clamp battery level to a minimum of 0%
        if self.battery_level < 0.0:
            self.battery_level = 0.0

    def timer_callback(self):
        # Handle time-based charging
        if self.is_charging:
            self.battery_level += self.charge_rate
            if self.battery_level > 100.0:
                self.battery_level = 100.0
            self.get_logger().info(f"Charging... Battery: {self.battery_level:.1f}%")

        # Publish the battery status for the Mission Executive to read later
        msg = Float32()
        msg.data = self.battery_level
        self.battery_pub.publish(msg)

        # Print a warning if battery gets critically low
        if self.battery_level < 20.0 and not self.is_charging:
            self.get_logger().warn(f"LOW BATTERY: {self.battery_level:.1f}%")

def main(args=None):
    rclpy.init(args=args)
    node = BatterySimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()