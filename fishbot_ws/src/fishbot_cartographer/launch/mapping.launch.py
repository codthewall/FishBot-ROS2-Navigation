import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取两个包的路径
    fishbot_cartographer_dir = get_package_share_directory('fishbot_cartographer')
    
    # 关键：获取 navigation2 包的路径来加载 ekf.yaml
    try:
        fishbot_navigation2_dir = get_package_share_directory('fishbot_navigation2')
    except:
        # 如果包名不同，请修改这里
        fishbot_navigation2_dir = os.path.join(
            os.path.expanduser('~'), 
            'chapt9/fishbot_ws/src/fishbot_navigation2'
        )
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='fishbot_2d.lua')

    cartographer_config_dir = os.path.join(fishbot_cartographer_dir, 'config')
    rviz_config_file = os.path.join(fishbot_cartographer_dir, 'rviz', 'cartographer.rviz')
    

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_configuration_basename_cmd = DeclareLaunchArgument(
        'configuration_basename',
        default_value='fishbot_2d.lua',
        description='Name of lua configuration file')

    # EKF 节点：将 /odom_raw 转换为 /odom
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=["/home/syf/chapt9/fishbot_ws/src/fishbot_navigation2/config/ekf.yaml"],
        remappings=[
            ('odom0', '/odom_raw'),  # 输入：ESP32 原始里程计
            ('odom', '/odom')        # 输出：融合后的里程计
        ]
    )

    # Cartographer 节点（订阅 EKF 发布的 /odom）
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', configuration_basename
        ],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odom_raw')  # 明确订阅 /odom（EKF 输出）
        ]
    )

    # 栅格地图节点
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', '0.05']
    )

    # RViz 节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_configuration_basename_cmd)
    ld.add_action(ekf_node)
    ld.add_action(cartographer_node)
    ld.add_action(occupancy_grid_node)
    ld.add_action(rviz_node)

    return ld