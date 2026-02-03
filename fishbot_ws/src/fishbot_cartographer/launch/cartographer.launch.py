import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取包的共享目录
    fishbot_cartographer_dir = get_package_share_directory('fishbot_cartographer')

    # 声明launch参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Cartographer配置文件路径
    cartographer_config_dir = os.path.join(fishbot_cartographer_dir, 'config')
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='fishbot_2d.lua')

    # 声明launch参数
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_configuration_basename_cmd = DeclareLaunchArgument(
        'configuration_basename',
        default_value='fishbot_2d.lua',
        description='Name of lua configuration file')

    # Cartographer节点
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
            ('scan', 'scan'),
        ]
    )

    # Cartographer占用栅格地图节点
    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
    )

    # 启动描述
    ld = LaunchDescription()

    # 添加声明的参数
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_configuration_basename_cmd)

    # 添加节点
    ld.add_action(cartographer_node)
    ld.add_action(cartographer_occupancy_grid_node)

    return ld
