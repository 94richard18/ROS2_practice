#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import time
import math

class TurtleControllerNode(Node):

    def __init__(self):
        super().__init__("Pose_Subsciber")
        self.get_logger().info("Turtle Controller has been started")
        self.state = 0          
        self.cmd_vel_pub_ = self.create_publisher(Twist,"/turtle1/cmd_vel",10)
        self.pose_subciber_ = self.create_subscription(Pose,"/turtle1/pose",self.pose_callback,1)

    def pose_callback(self,pose: Pose):

        cmd = Twist()
        if self.state == 0:
            cmd.linear.x = 2.0
            cmd.angular.z = 0.0
            self.state = 1

        else:
            cmd.linear.x = math.pi/4
            cmd.angular.z = math.pi/2
            self.state = 0
            

        self.cmd_vel_pub_.publish(cmd)
        #time.sleep(2)
def main(args=None):
    rclpy.init(args=args)

    node = TurtleControllerNode()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main() 