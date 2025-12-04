"""
带3D感知的MoveIt2 Demo Launch文件
启动demo + 点云避障感知
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('mmk2_moveit_config')
    
    # 声明参数
    voxel_size_arg = DeclareLaunchArgument(
        'voxel_size', default_value='0.01', description='体素大小(米)'
    )
    max_depth_arg = DeclareLaunchArgument(
        'max_depth', default_value='1.0', description='最大感知距离(米)'
    )
    update_rate_arg = DeclareLaunchArgument(
        'update_rate', default_value='1.0', description='障碍物更新频率(Hz)'
    )
    
    # 启动原始demo
    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_share, 'launch', 'demo.launch.py'])
        ])
    )
    
    # 点云生成节点
    pointcloud_node = Node(
        package='mmk2_moveit_config',
        executable='depth_to_pointcloud_node.py',
        name='depth_to_pointcloud_node',
        output='screen',
        parameters=[{
            'depth_topic': '/head_camera/aligned_depth_to_color/image_raw',
            'color_topic': '/head_camera/color/image_raw',
            'camera_info_topic': '/head_camera/color/camera_info',
            'pointcloud_topic': '/head_camera/pointcloud',
            'frame_id': 'head_camera_link',
            'depth_scale': 0.001,
            'max_depth': LaunchConfiguration('max_depth'),
            'min_depth': 0.1,
            'downsample_factor': 2,
        }]
    )
    
    # 点云转障碍物节点（延迟启动，等待MoveIt初始化）
    obstacle_node = TimerAction(
        period=5.0,  # 延迟5秒启动
        actions=[
            Node(
                package='mmk2_moveit_config',
                executable='pointcloud_to_planning_scene.py',
                name='pointcloud_to_planning_scene',
                output='screen',
                parameters=[{
                    'pointcloud_topic': '/head_camera/pointcloud',
                    'planning_scene_topic': '/planning_scene',
                    'voxel_size': LaunchConfiguration('voxel_size'),
                    'min_points_per_voxel': 2,
                    'ground_height': 0.02,
                    'max_height': 1.5,
                    'min_height': 0.0,
                    'frame_id': 'base_link',
                    'update_rate': LaunchConfiguration('update_rate'),
                    'obstacle_padding': 0.005,
                    'use_boxes': False,
                    'cluster_tolerance': 0.02,
                    'max_distance': 1.0,
                }]
            )
        ]
    )
    
    return LaunchDescription([
        voxel_size_arg,
        max_depth_arg,
        update_rate_arg,
        demo_launch,
        pointcloud_node,
        obstacle_node,
    ])
