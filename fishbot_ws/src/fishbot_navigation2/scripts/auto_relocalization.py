#!/usr/bin/env python3
"""
自动重定位检测节点
检测里程计与激光雷达匹配结果之间的偏差，当偏移超过阈值时自动触发重定位
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Trigger
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math
import time


class AutoRelocalization(Node):
    def __init__(self):
        super().__init__('auto_relocalization')
        
        # 参数
        self.declare_parameter('check_interval', 2.0)  # 检查间隔（秒）
        self.declare_parameter('drift_threshold', 0.5)  # 漂移阈值（米）
        self.declare_parameter('angle_drift_threshold', 0.3)  # 角度漂移阈值（弧度）
        self.declare_parameter('min_velocity', 0.05)  # 最小速度阈值
        
        self.check_interval = self.get_parameter('check_interval').value
        self.drift_threshold = self.get_parameter('drift_threshold').value
        self.angle_drift_threshold = self.get_parameter('angle_drift_threshold').value
        self.min_velocity = self.get_parameter('min_velocity').value
        
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 状态
        self.last_odom_time = None
        self.last_pose = None
        self.initialized = False
        self.last_check_time = time.time()
        
        # 订阅里程计
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # 发布重定位话题（用于手动调整）
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        
        # 重定位服务客户端
        self.relocalize_client = self.create_client(Trigger, '/trigger_global_localization')
        
        # 计时器
        self.timer = self.create_timer(self.check_interval, self.check_timer_callback)
        
        self.get_logger().info('自动重定位检测节点已启动')
        self.get_logger().info(f'漂移阈值: {self.drift_threshold}m, {self.angle_drift_threshold}rad')

    def odom_callback(self, msg):
        """处理里程计数据"""
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        if not self.initialized:
            self.last_odom_time = current_time
            self.last_pose = (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                self.get_yaw(msg.pose.pose.orientation)
            )
            self.initialized = True
            return
        
        # 计算时间差和位置变化
        dt = current_time - self.last_odom_time
        if dt < 0.01:  # 避免除零
            return
            
        dx = msg.pose.pose.position.x - self.last_pose[0]
        dy = msg.pose.pose.position.y - self.last_pose[1]
        dyaw = self.get_yaw(msg.pose.pose.orientation) - self.last_pose[2]
        
        # 归一化角度变化
        while dyaw > math.pi:
            dyaw -= 2 * math.pi
        while dyaw < -math.pi:
            dyaw += 2 * math.pi
            
        # 计算速度
        vx = dx / dt
        vy = dy / dt
        v = math.sqrt(vx * vx + vy * vy)
        
        # 更新状态
        self.last_odom_time = current_time
        self.last_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            self.get_yaw(msg.pose.pose.orientation)
        )
        
        # 如果速度太低，跳过检查
        if v < self.min_velocity:
            return

    def check_timer_callback(self):
        """定时检查是否需要重定位"""
        if not self.initialized:
            return
            
        # 检查TF变换
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'map', 'base_link',
                now,
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            
            # 检查位置偏移
            tx = trans.transform.translation.x
            ty = trans.transform.translation.y
            yaw = self.get_yaw(trans.transform.rotation)
            
            # 计算偏移量（相对于原点，因为GICP会在原点附近搜索）
            offset = math.sqrt(tx * tx + ty * ty)
            abs_yaw = abs(yaw)
            
            current_time = time.time()
            
            # 如果偏移超过阈值，触发重定位
            if offset > self.drift_threshold or abs_yaw > self.angle_drift_threshold:
                if current_time - self.last_check_time > 10.0:  # 避免频繁触发
                    self.get_logger().warn(
                        f'检测到定位偏移: 位置={offset:.2f}m, 角度={abs_yaw:.2f}rad，触发重定位...'
                    )
                    self.trigger_relocalization()
                    self.last_check_time = current_time
                else:
                    self.get_logger().info(
                        f'检测到定位偏移但跳过: 位置={offset:.2f}m, 角度={abs_yaw:.2f}rad'
                    )
            else:
                self.get_logger().debug(f'定位正常: 偏移={offset:.2f}m')
                
        except TransformException as e:
            self.get_logger().debug(f'TF查询失败: {e}')

    def trigger_relocalization(self):
        """触发全局重定位"""
        if not self.relocalize_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('重定位服务不可用')
            return
            
        request = Trigger.Request()
        future = self.relocalize_client.call_async(request)
        
        def callback(response):
            if response.success:
                self.get_logger().info('重定位已触发，等待定位完成...')
            else:
                self.get_logger().error(f'重定位触发失败: {response.message}')
                
        future.add_done_callback(callback)

    @staticmethod
    def get_yaw(quaternion):
        """从四元数提取偏航角"""
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = AutoRelocalization()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()



