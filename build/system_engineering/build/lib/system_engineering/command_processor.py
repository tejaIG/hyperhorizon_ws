#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time

class CommandProcessor(Node):
    def __init__(self):
        super().__init__('command_processor')
        
        # Create subscription to commands topic
        self.subscription = self.create_subscription(
            String,
            'commands',
            self.command_callback,
            10
        )
        
        # Create publisher for velocity commands
        self.publisher_ = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        
        self.get_logger().info('Command Processor node has been started')
        
    def command_callback(self, msg):
        self.get_logger().info(f'Received command: {msg.data}')
        
        # Process the command and create appropriate Twist message
        twist_msg = Twist()
        
        if msg.data == "MOVE_FORWARD":
            twist_msg.linear.x = 0.5
            twist_msg.angular.z = 0.0
        elif msg.data == "MOVE_BACKWARD":
            twist_msg.linear.x = -0.5
            twist_msg.angular.z = 0.0
        elif msg.data == "TURN_LEFT":
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.5
        elif msg.data == "TURN_RIGHT":
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = -0.5
        elif msg.data == "STOP":
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
        
        # Publish the twist message
        self.publisher_.publish(twist_msg)
        self.get_logger().info(f'Published velocity: linear.x={twist_msg.linear.x}, angular.z={twist_msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    command_processor = CommandProcessor()
    
    try:
        rclpy.spin(command_processor)
    except KeyboardInterrupt:
        command_processor.get_logger().info('Command Processor node stopped')
    finally:
        command_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
