#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random
import time

class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher')
        self.publisher_ = self.create_publisher(String, 'commands', 10)
        self.timer = self.create_timer(2.0, self.publish_command)
        self.get_logger().info('Command Publisher node has been started')
        
        # Available commands
        self.commands = [
            "MOVE_FORWARD",
            "MOVE_BACKWARD",
            "TURN_LEFT",
            "TURN_RIGHT",
            "STOP"
        ]
    
    def publish_command(self):
        msg = String()
        command = random.choice(self.commands)
        msg.data = command
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing command: {command}')

def main(args=None):
    rclpy.init(args=args)
    command_publisher = CommandPublisher()
    
    try:
        rclpy.spin(command_publisher)
    except KeyboardInterrupt:
        command_publisher.get_logger().info('Command Publisher node stopped')
    finally:
        command_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
