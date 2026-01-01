#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class SmartSensor(Node):
    def __init__(self):
        super().__init__('smart_sensor')
        
        # Publisher to send commands to Motor
        self.publisher_ = self.create_publisher(String, 'motor_command', 10)
        
        # Subscriber to receive status from Motor
        self.subscription = self.create_subscription(
            String,
            'motor_status',
            self.listener_callback,
            10)
        self.subscription  # preventative
        
        self.get_logger().info('Smart Sensor Node Started.')
        
        # Start the cycle automatically after 2 seconds to ensure connections are ready
        self.start_timer_ = self.create_timer(2.0, self.start_cycle)

    def start_cycle(self):
        # Cancel the start timer so it doesn't repeat
        self.start_timer_.cancel()
        
        msg = String()
        msg.data = "Start Motor"
        self.publisher_.publish(msg)
        self.get_logger().info('Sent Command: "Start Motor"')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received Status: "{msg.data}"')
        
        if msg.data == "Completed":
            self.get_logger().info('Cycle finish. Waiting 5 seconds to restart...')
            # Create a one-shot timer to restart the cycle
            self.restart_timer_ = self.create_timer(5.0, self.restart_cycle)

    def restart_cycle(self):
        self.restart_timer_.cancel() # One-shot
        self.start_cycle()

def main(args=None):
    rclpy.init(args=args)
    node = SmartSensor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
