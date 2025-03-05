#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class TurtleControllerNode(Node):

    def __init__(self):
        super().__init__("Pose_Subsciber")
        self.get_logger().info("Turtle Controller has been started")
        self.cmd_vel_pub_ = self.create_publisher(Twist,"/turtle1/cmd_vel",10)
        self.pose_subciber_ = self.create_subscription(Pose,"/turtle1/pose",self.pose_callback,10)

    def pose_callback(self,pose: Pose):

        cmd = Twist()

        if pose.x >9.0 or pose.x<2.0 or pose.y > 9.0 or pose.y <2.0:
            cmd.linear.x = 0.5
            cmd.angular.z = 0.5
        else:
            cmd.linear.x = 10.0
            cmd.angular.z = 0.1
        self.cmd_vel_pub_.publish(cmd)
def main(args=None):
    rclpy.init(args=args)

    node = TurtleControllerNode()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main() 