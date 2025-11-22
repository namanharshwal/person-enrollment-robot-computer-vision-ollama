#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        self.subscription = self.create_subscription(
            String,  # Subscribe to gesture commands
            'gesture_command',
            self.gesture_callback,
            10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info("Navigation Controller Node Started")
        
    def gesture_callback(self, msg):
        self.get_logger().info(f"Received gesture: {msg.data}")
        twist = Twist()
        
        if msg.data == "thumbs_up":
            # Move forward
            twist.linear.x = 0.5
            self.get_logger().info("Moving forward")
        elif msg.data == "stop":
            # Stop
            self.get_logger().info("Stopping")
        elif msg.data == "point_left":
            # Turn left
            twist.angular.z = 0.5
            self.get_logger().info("Turning left")
        elif msg.data == "point_right":
            # Turn right
            twist.angular.z = -0.5
            self.get_logger().info("Turning right")
        elif msg.data == "palm_open":
            # Move backward
            twist.linear.x = -0.3
            self.get_logger().info("Moving backward")
            
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
