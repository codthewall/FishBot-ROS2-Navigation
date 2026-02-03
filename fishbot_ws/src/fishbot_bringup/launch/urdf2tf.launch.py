import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    生成Launch描述，用于加载fishbot机器人模型
    """
    
    # ======================================
    # 1. 获取URDF模型的默认路径
    # ======================================
    urdf_package_path = get_package_share_directory('fishbot_description')
    default_model_path = os.path.join(urdf_package_path, 'urdf', 'fishbot.urdf')
    
    # ======================================
    # 2. 声明Launch参数
    # ======================================
    declare_model_arg = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=str(default_model_path),
        description='Absolute path to robot URDF file'
    )
    
    # ======================================
    # 3. 读取URDF文件内容
    # ======================================
    robot_description_param = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(['cat ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str
    )
    
    # ======================================
    # 4. 创建机器人状态发布节点
    # ======================================
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_param}]
    )
    
    # ======================================
    # 5. 创建关节状态发布节点
    # ======================================
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )
    
    # ======================================
    # 6. 创建RViz2节点（可选）
    # ======================================
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )
    
    # ======================================
    # 7. 返回Launch描述
    # ======================================
    return launch.LaunchDescription([
        declare_model_arg,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])