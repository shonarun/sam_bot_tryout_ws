from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory(package='sam_bot_description').find('sam_bot_description')
    default_model_path = os.path.join(pkg_share, 'src', 'description', 'sam_bot_description.sdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'config.rviz')
    bridge_config_path = os.path.join(pkg_share, 'config', 'bridge_config.yaml')

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_base',
        arguments=[
            '0.1', '0.0', '0.2',
            '0.0', '0.0', '0.0', '1.0',
            'base_link', 'base_laser'
        ]
    )
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot model file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),
        robot_state_publisher_node,
        rviz_node,
        static_tf
    ])