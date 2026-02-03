#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
import sys

class AutoLocalization(Node):
    def __init__(self):
        super().__init__('auto_localization')
        
        # ========== 第1步：设置地图原点为初始位姿 ==========
        self.get_logger().info('=== Step 1: Setting initial pose to map origin (0,0) ===')
        self.set_initial_pose_to_origin()
        
        # 等待 Cartographer 响应并发布 map frame
        self.get_logger().info('Waiting for Cartographer to publish map frame...')
        time.sleep(3.0)  # 给 Cartographer 时间处理
        
        # ========== 第2步：调用 GICP 进行全局定位纠正 ==========
        self.get_logger().info('=== Step 2: Triggering GICP global localization ===')
        self.call_gicp_service()
        
        rclpy.shutdown()

    def set_initial_pose_to_origin(self):
        """发布 /initialpose 让 Cartographer 以原点初始化"""
        pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # 设置地图原点 (0,0,0)
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        
        # 协方差矩阵（表示不确定性较大，允许快速收敛）
        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,  # x 方差 0.5m
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,  # y 方差 0.5m
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1   # yaw 方差
        ]
        
        # 多次发布确保送达
        for i in range(5):
            msg.header.stamp = self.get_clock().now().to_msg()
            pub.publish(msg)
            self.get_logger().info(f'Published origin pose ({i+1}/5)')
            time.sleep(0.3)

    def call_gicp_service(self):
        """调用 GICP 服务进行全局定位"""
        self.cli = self.create_client(Trigger, '/trigger_global_localization')
        
        # 等待 GICP 服务就绪
        self.get_logger().info('Waiting for GICP service...')
        max_retries = 30
        retry = 0
        while not self.cli.wait_for_service(timeout_sec=1.0):
            retry += 1
            if retry > max_retries:
                self.get_logger().error('GICP service timeout, please check if gicp_relocalizer is running')
                return
            self.get_logger().info(f'Service not ready, retrying {retry}/{max_retries}...')
        
        # 调用服务
        self.req = Trigger.Request()
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        
        if self.future.result() is not None:
            if self.future.result().success:
                self.get_logger().info('✓ GICP localization successful! Pose corrected.')
            else:
                self.get_logger().warn('✗ GICP localization failed, please set pose manually in RViz')
        else:
            self.get_logger().error('Service call failed')

if __name__ == '__main__':
    rclpy.init()
    try:
        AutoLocalization()
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)