import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 获取路径
    fishbot_navigation2_dir = get_package_share_directory('fishbot_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    gicp_relocalizer_dir = get_package_share_directory('gicp_relocalizer')
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

    # Launch 配置
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml_path = LaunchConfiguration(
        'map', default=os.path.join(fishbot_navigation2_dir, 'maps', 'my_map.yaml'))
    nav2_param_path = LaunchConfiguration(
        'params_file', default=os.path.join(fishbot_navigation2_dir, 'config', 'nav2_params.yaml'))
    ekf_param_path = LaunchConfiguration(
        'ekf_params_file', default=os.path.join(fishbot_navigation2_dir, 'config', 'ekf.yaml'))
    gicp_param_path = LaunchConfiguration(
        'gicp_params', default=os.path.join(gicp_relocalizer_dir, 'config', 'gicp_params.yaml'))

    return launch.LaunchDescription([
        # 声明参数
        launch.actions.DeclareLaunchArgument(
            'use_sim_time', default_value=use_sim_time,
            description='Use simulation clock if true'),
        launch.actions.DeclareLaunchArgument(
            'map', default_value=map_yaml_path,
            description='Full path to map file'),
        launch.actions.DeclareLaunchArgument(
            'params_file', default_value=nav2_param_path,
            description='Full path to nav2 params file'),
        launch.actions.DeclareLaunchArgument(
            'ekf_params_file', default_value=ekf_param_path,
            description='Full path to EKF params file'),
        launch.actions.DeclareLaunchArgument(
            'gicp_params', default_value=gicp_param_path,
            description='Full path to GICP params file'),

        # 0. 静态 TF（base_link -> laser_link）
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_tf_pub',
            arguments=['0.1', '0', '0.2', '0', '0', '0', 'base_link', 'laser_link'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # 1. 地图服务器
        launch_ros.actions.Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml_path,
                         'use_sim_time': use_sim_time}]
        ),

        # 2. 生命周期管理器（地图）
        launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'autostart': True,
                         'node_names': ['map_server']}]
        ),

        # 3. EKF 节点
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_param_path],
            remappings=[
                ('odom0', '/odom_raw'),
                ('odom', '/odom'),
                ('odometry/filtered', '/odom')
            ]
        ),

        # 4. Cartographer 纯定位（以原点启动）
        launch_ros.actions.Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', os.path.join(fishbot_navigation2_dir, 'config'),
                '-configuration_basename', 'fishbot_2d_localization.lua',
                '-pure_localization', 'true'
                # 注意：没有 -load_state，Cartographer 以原点初始化
            ],
            remappings=[
                ('scan', '/scan'),
                ('odom', '/odom'),
            ]
        ),

        # 5. ========== GICP 重定位节点（关键新增） ==========
        # 等待 Cartographer 稳定并发布 map frame
        TimerAction(
            period=10.0,  # 增加到10秒，确保Cartographer完全初始化
            actions=[
                launch_ros.actions.Node(
                    package='gicp_relocalizer',
                    executable='gicp_relocalizer',
                    name='gicp_relocalizer',
                    output='screen',
                    parameters=[gicp_param_path],
                    remappings=[
                        ('/scan', '/scan'),
                        ('/map', '/map'),
                        ('/odom', '/odom'),
                        ('/initialpose', '/initialpose'),
                    ],
                ),
            ]
        ),

        # 6. ========== 自动触发 GICP 全局定位 ==========
        # 等待足够长时间确保激光数据已接收和GICP初始化完成
        TimerAction(
            period=20.0,  # 增加到20秒，确保所有组件就绪
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'service', 'call', '/trigger_global_localization',
                         'std_srvs/srv/Trigger', '{}'],
                    output='screen',
                    name='trigger_gicp_localization'
                )
            ]
        ),

        # 7. Nav2 导航组件（等待 GICP 定位完成后启动）
        TimerAction(
            period=25.0,  # 增加到25秒，给GICP足够时间完成定位
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        'params_file': nav2_param_path,
                        'autostart': 'True',
                    }.items(),
                )
            ]
        ),

        # 8. ========== 自动重定位检测节点 ==========
        # 当检测到定位偏移时自动触发重定位
        launch_ros.actions.Node(
            package='fishbot_navigation2',
            executable='auto_relocalization.py',
            name='auto_relocalization',
            output='screen',
            parameters=[{
                'check_interval': 2.0,
                'drift_threshold': 0.5,      # 位置漂移阈值（米）
                'angle_drift_threshold': 0.3, # 角度漂移阈值（弧度）
                'min_velocity': 0.05,         # 最小速度阈值
            }]
        ),

        # 9. RViz
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])