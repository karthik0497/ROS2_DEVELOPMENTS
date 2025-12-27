#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

class MyCustomNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info(f'Node {node_name} has started.')
        
        # Create a timer (0.5s)
        self.timer_ = self.create_timer(0.5, self.timer_callback)
        self.counter_ = 0
    def timer_callback(self):
        self.counter_ += 1
        self.get_logger().info(f'Hello from {self.get_name()} | Count: {self.counter_}')




def test_node_a(args=None):
    rclpy.init(args=args)
    node = MyCustomNode('test_node_a')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

def test_node_b(args=None):
    rclpy.init(args=args)
    node = MyCustomNode('test_node_b')
    rclpy.spin(node)
    node.destroy_node() 
    rclpy.shutdown()

def test_node_c(args=None):
    rclpy.init(args=args)
    node = MyCustomNode('test_node_c')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()