"""
点云避障感知Launch文件
将点云转换为MoveIt障碍物，与demo.launch.py配合使用

使用方法：
1. 先启动demo.launch.py
2. 再启动此launch文件
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 声明参数
    pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='/head_camera/pointcloud',
        description='输入点云话题'
    )
    
    voxel_size_arg = DeclareLaunchArgument(
        'voxel_size',
        default_value='0.05',
        description='体素大小(米)'
    )
    
    ground_height_arg = DeclareLaunchArgument(
        'ground_height',
        default_value='0.05',
        description='地面过滤高度(米)'
    )
    
    max_height_arg = DeclareLaunchArgument(
        'max_height',
        default_value='2.0',
        description='最大感知高度(米)'
    )
    
    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='2.0',
        description='障碍物更新频率(Hz)'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='base_link',
        description='障碍物坐标系'
    )
    
    # 点云生成节点（从深度相机）
    depth_to_pointcloud_node = Node(
        package='mmk2_moveit_config',
        executable='depth_to_pointcloud_node.py',
        name='depth_to_pointcloud_node',
        output='screen',
        parameters=[{
            'depth_topic': '/head_camera/aligned_depth_to_color/image_raw',
            'color_topic': '/head_camera/color/image_raw',
            'camera_info_topic': '/head_camera/color/camera_info',
            'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
            'frame_id': 'head_camera_link',
            'depth_scale': 0.001,
            'max_depth': 3.0,
            'min_depth': 0.1,
            'downsample_factor': 2,
        }]
    )
    
    # 点云转PlanningScene节点
    pointcloud_to_planning_scene_node = Node(
        package='mmk2_moveit_config',
        executable='pointcloud_to_planning_scene.py',
        name='pointcloud_to_planning_scene',
        output='screen',
        parameters=[{
            'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
            'planning_scene_topic': '/planning_scene',
            'voxel_size': LaunchConfiguration('voxel_size'),
            'min_points_per_voxel': 3,
            'ground_height': LaunchConfiguration('ground_height'),
            'max_height': LaunchConfiguration('max_height'),
            'min_height': 0.0,
            'frame_id': LaunchConfiguration('frame_id'),
            'update_rate': LaunchConfiguration('update_rate'),
            'obstacle_padding': 0.02,
            'use_boxes': True,
            'cluster_tolerance': 0.1,
        }]
    )
    
    return LaunchDescription([
        pointcloud_topic_arg,
        voxel_size_arg,
        ground_height_arg,
        max_height_arg,
        update_rate_arg,
        frame_id_arg,
        depth_to_pointcloud_node,
        pointcloud_to_planning_scene_node,
    ])
