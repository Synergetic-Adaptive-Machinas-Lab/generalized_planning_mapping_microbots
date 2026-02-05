#!/usr/bin/env python3

"""
ROS2 Launch file for Microbot Detector
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for microbot detector"""
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('microbot_detector_ros2'),
            'config',
            'detector_params.yaml'
        ]),
        description='Path to the detector parameters YAML file'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to start RViz'
    )
    
    # Microbot detector node
    detector_node = Node(
        package='microbot_detector_ros2',
        executable='microbot_detector_node.py',
        name='microbot_detector',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        emulate_tty=True
    )
    
    # RViz node (conditional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )
    
    return LaunchDescription([
        config_file_arg,
        use_rviz_arg,
        detector_node,
        # rviz_node  # Uncomment if you want RViz support
    ])
