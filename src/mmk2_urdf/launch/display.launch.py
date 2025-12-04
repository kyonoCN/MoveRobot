#!/usr/bin/env python3
"""
Launch file for displaying MMK2 robot in RViz.
Usage: ros2 launch mmk2_urdf display.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get package directory
    pkg_share = get_package_share_directory('mmk2_urdf')
    
    # Paths
    urdf_file = os.path.join(pkg_share, 'urdf', 'mmk2.urdf')
    rviz_config_file = os.path.join(pkg_share, 'config', 'display.rviz')
    
    # Launch arguments
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Use joint_state_publisher_gui instead of joint_state_publisher'
    )
    
    # Read URDF file
    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )
    
    # Joint State Publisher GUI
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else []
    )
    
    return LaunchDescription([
        use_gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
