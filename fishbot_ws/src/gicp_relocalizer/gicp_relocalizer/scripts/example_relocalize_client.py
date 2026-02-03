#!/usr/bin/env python3
"""
示例：如何在 gicp_relocalizer 中使用 Cartographer 的重定位服务

这个示例展示了在收到 GICP 重定位结果后，如何调用 Cartographer 的 
/relocalize 服务来更新位姿。
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from cartographer_ros_msgs.srv import Relocalize
from std_msgs.msg import Header


class GICPRelocalizer(Node):
    def __init__(self):
        super().__init__('gicp_relocalizer')
        
        # 创建 Cartographer 重定位服务的客户端
        self.relocalize_client = self.create_client(
            Relocalize, 
            '/relocalize'
        )
        
        # 等待服务可用
        while not self.relocalize_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/relocalize 服务不可用，等待中...')
        
        self.get_logger().info('/relocalize 服务已连接')
        
        # 订阅 GICP 重定位结果话题（根据你的实际话题名调整）
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/gicp_pose',  # 替换为你的 GICP 话题名
            self.gicp_pose_callback,
            10
        )
        
        # 存储上一次发送的位姿，避免重复发送
        self.last_sent_pose = None
        
    def gicp_pose_callback(self, msg: PoseWithCovarianceStamped):
        """收到 GICP 重定位结果时调用"""
        
        # 检查位姿是否有效
        if not self.is_pose_valid(msg.pose.pose):
            self.get_logger().warning('收到无效的 GICP 位姿')
            return
        
        # 可选：添加简单的变化检测，避免重复发送相同的位姿
        if self.last_sent_pose is not None:
            if self.pose_equal(msg.pose.pose, self.last_sent_pose):
                return
        
        # 发送位姿到 Cartographer
        self.send_pose_to_cartographer(0, msg.pose.pose)  # trajectory_id 通常是 0
        self.last_sent_pose = msg.pose.pose
        
    def send_pose_to_cartographer(self, trajectory_id: int, pose: Pose):
        """发送位姿到 Cartographer 进行重定位"""
        
        request = Relocalize.Request()
        request.trajectory_id = trajectory_id
        request.pose = pose
        
        self.get_logger().info(
            f'发送重定位请求: trajectory_id={trajectory_id}, '
            f'pose=({pose.position.x:.2f}, {pose.position.y:.2f}, '
            f'{pose.position.z:.2f})'
        )
        
        # 异步调用服务
        future = self.relocalize_client.call_async(request)
        
        # 添加回调处理响应
        future.add_done_callback(self.relocalize_response_callback)
        
    def relocalize_response_callback(self, future):
        """处理重定位服务响应"""
        try:
            response = future.result()
            if response.status.code == 0:  # SUCCESS
                self.get_logger().info('重定位成功！')
            else:
                self.get_logger().error(
                    f'重定位失败: {response.status.message}'
                )
        except Exception as e:
            self.get_logger().error(f'重定位服务调用失败: {e}')
    
    @staticmethod
    def is_pose_valid(pose: Pose) -> bool:
        """检查位姿是否有效"""
        # 检查位置是否为 NaN
        if (pose.position.x != pose.position.x or 
            pose.position.y != pose.position.y or 
            pose.position.z != pose.position.z):
            return False
        
        # 检查四元数是否归一化
        norm = (pose.orientation.x ** 2 + 
                pose.orientation.y ** 2 + 
                pose.orientation.z ** 2 + 
                pose.orientation.w ** 2) ** 0.5
        
        if abs(norm - 1.0) > 0.01:
            return False
            
        return True
    
    @staticmethod
    def pose_equal(pose1: Pose, pose2: Pose, threshold: float = 0.01) -> bool:
        """检查两个位姿是否相等（在阈值内）"""
        pos_diff = ((pose1.position.x - pose2.position.x) ** 2 +
                    (pose1.position.y - pose2.position.y) ** 2 +
                    (pose1.position.z - pose2.position.z) ** 2) ** 0.5
        
        return pos_diff < threshold


def main(args=None):
    rclpy.init(args=args)
    
    relocalizer = GICPRelocalizer()
    
    try:
        rclpy.spin(relocalizer)
    except KeyboardInterrupt:
        pass
    finally:
        relocalizer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


