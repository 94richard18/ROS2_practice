import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
import math
import numpy as np  # 用來計算平均值

class SmartObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('smart_obstacle_avoidance')
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.toggle_service = self.create_service(SetBool, '/toggle_robot', self.toggle_callback)
        self.robot_active = False

        # 參數化設置
        self.declare_parameter('max_speed', 0.5)
        self.declare_parameter('min_distance', 0.3)
        self.max_speed = self.get_parameter('max_speed').value
        self.min_distance = self.get_parameter('min_distance').value

        self.get_logger().info("🤖 Smart Obstacle Avoidance Node Started!")

    def scan_callback(self, msg):
        """處理雷射數據以進行更智能的避障"""
        if not self.robot_active or not msg.ranges:
            return

        # 取出前方 ±30° 的距離數據，計算平均值
        front_distances = msg.ranges[150:210]
        front_distance = np.nanmin(front_distances) if front_distances else float('inf')

        # 取得左右側的距離數據
        left_distances = msg.ranges[0:120]
        left_distance = np.nanmean(left_distances) if left_distances else float('inf')

        right_distances = msg.ranges[240:359]
        right_distance = np.nanmean(right_distances) if right_distances else float('inf')

        cmd = Twist()

        # 避障邏輯
        if front_distance < self.min_distance:
            # 若左右空間均充足，轉向較開放的那一側
            if left_distance > right_distance:
                cmd.angular.z = math.pi/8  # 左轉
            elif math.isinf(left_distance) and right_distance < 0.3: 
                cmd.angular.z = math.pi/2
            elif math.isinf(right_distance) and left_distance < 0.3: 
                cmd.angular.z = -math.pi/2     
            elif math.isinf(left_distance) and math.isinf(right_distance):
                cmd.angular.z  = math.pi/8          
            else:
                cmd.angular.z = -math.pi/4 # 右轉
            cmd.linear.x = -0.01  # 停止向前
        else:
            # 如果左右空間接近，則直走
            if abs(left_distance - right_distance) == 0.0:
                cmd.linear.x = self.max_speed
                cmd.angular.z = 0.0        
            else:
                # 若牆壁在左側，讓機器人稍微右轉；反之則左轉
                cmd.linear.x = self.max_speed
                cmd.angular.z = -0.3 if left_distance > right_distance else 0.3

        self.cmd_vel_publisher.publish(cmd)
        self.get_logger().info(f"Front: {front_distance:.2f}m, Left: {left_distance:.2f}m, Right: {right_distance:.2f}m")

    def toggle_callback(self, request, response):
        """開關機器人的 Service 回調函數"""
        cmd = Twist()
        if request.data:
            self.robot_active = True
            response.message = "Robot is now ACTIVE!"
        else:
            self.robot_active = False
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_publisher.publish(cmd)
            response.message = "Robot is now STOPPED!"

        response.success = True
        self.get_logger().info(response.message)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = SmartObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
