#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from actuator_msgs.msg import Actuators

class TakeOffNode(Node):
    
    def __init__(self):
        super().__init__('take_off_controller')
        
        # Subscribe to drone pose (position + orientation)
        self.pose_subscriber = self.create_subscription(
            PoseArray,
            '/model/parrot_bebop_2/pose',
            self.read_sensors,
            10)
        
        # Publish motor speed commands
        self.motor_speed_publisher = self.create_publisher(
            Actuators,
            '/parrot_bebop_2/command/motor_speed',
            10)
        
    def read_sensors(self , pose_msg):
        print(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TakeOffNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
