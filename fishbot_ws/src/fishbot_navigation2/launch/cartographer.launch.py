import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    fishbot_navigation2_dir = get_package_share_directory('fishbot_navigation2')
    
    return LaunchDescription([
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[
                '-configuration_directory', os.path.join(fishbot_navigation2_dir, 'config'),
                '-configuration_basename', 'fishbot_2d_localization.lua',
                '-pure_localization', 'true'
            ],
            remappings=[
                ('scan', '/scan'),
                ('odom', '/odom'),
            ]
        ),
    ])










