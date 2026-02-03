import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 1. 获取功能包路径
    fishbot_bringup_dir = get_package_share_directory('fishbot_bringup')
    ydlidar_ros2_dir = get_package_share_directory('ydlidar')
    
    # 2. 加载 URDF 转 TF 的 launch 文件
    urdf2tf = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [fishbot_bringup_dir, '/launch/urdf2tf.launch.py']
        )
    )
    
    # 3. 启动 odom 转 TF 的节点
    # 注意：已注释掉，因为 EKF 会发布 odom → base_link
    # 如果没有 EKF，可以启用此节点
    # odom2tf = launch_ros.actions.Node(
    #     package='fishbot_bringup',
    #     executable='odom2tf',
    #     output='screen'
    # )
    # odom2tf.remappings = [('odom', '/odom_raw')]
    
    # 4. 启动 micro-ROS Agent 节点
    microros_agent = launch_ros.actions.Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        arguments=['udp4', '--port', '8888'],
        output='screen'
    )
    
    # 5. 启动串口转 WiFi 节点
    ros_serial2wifi = launch_ros.actions.Node(
        package='ros_serial2wifi',
        executable='tcp_server',
        parameters=[{'serial_port': '/tmp/tty_laser'}],
        output='screen'
    )
    
    # 6. 加载激光雷达驱动 launch 文件（延迟启动）
    ydlidar = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ydlidar_ros2_dir, '/launch', '/ydlidar_launch.py']
        )
    )
    
    # 7. 延迟 5 秒后启动激光雷达
    ydlidar_delay = launch.actions.TimerAction(
        period=5.0,
        actions=[ydlidar]
    )
    
    # 返回 LaunchDescription
    return launch.LaunchDescription([
        urdf2tf,
        # odom2tf,  # 已注释：EKF负责发布 odom → base_link
        microros_agent,
        ros_serial2wifi,
        ydlidar_delay
    ])