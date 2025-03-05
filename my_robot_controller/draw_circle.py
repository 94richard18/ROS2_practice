#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time 
import math

class DrawCircleNode(Node):

    def __init__(self):
        super().__init__("draw_circle")
        self.cmd_vel_pub_ = self.create_publisher(Twist,"/turtle1/cmd_vel",10)
        self.timer_ = self.create_timer(0.5,self.send_velocity_command)
        self.get_logger().info("Draw circle node has been started")

    def send_velocity_command(self):
        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = math.pi/2
        self.cmd_vel_pub_.publish(msg)
        time.sleep(2)

 
def main(args=None):
    rclpy.init(args=args)

    node = DrawCircleNode()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main() 