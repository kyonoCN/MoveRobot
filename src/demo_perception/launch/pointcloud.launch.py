"""
点云生成节点Launch文件
从深度图和RGB图像生成点云，基于head_camera_link坐标系
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 声明参数
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/head_camera/aligned_depth_to_color/image_raw',
        description='深度图话题'
    )
    
    color_topic_arg = DeclareLaunchArgument(
        'color_topic',
        default_value='/head_camera/color/image_raw',
        description='RGB图像话题'
    )
    
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/head_camera/color/camera_info',
        description='相机内参话题'
    )
    
    pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='/head_camera/pointcloud',
        description='输出点云话题'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='head_camera_link',
        description='点云坐标系'
    )
    
    depth_scale_arg = DeclareLaunchArgument(
        'depth_scale',
        default_value='0.001',
        description='深度值缩放因子(mm转m)'
    )
    
    max_depth_arg = DeclareLaunchArgument(
        'max_depth',
        default_value='5.0',
        description='最大深度(米)'
    )
    
    min_depth_arg = DeclareLaunchArgument(
        'min_depth',
        default_value='0.1',
        description='最小深度(米)'
    )
    
    downsample_factor_arg = DeclareLaunchArgument(
        'downsample_factor',
        default_value='2',
        description='下采样因子(减少点云数量)'
    )
    
    # 点云生成节点
    pointcloud_node = Node(
        package='mmk2_moveit_config',
        executable='depth_to_pointcloud_node.py',
        name='depth_to_pointcloud_node',
        output='screen',
        parameters=[{
            'depth_topic': LaunchConfiguration('depth_topic'),
            'color_topic': LaunchConfiguration('color_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
            'frame_id': LaunchConfiguration('frame_id'),
            'depth_scale': LaunchConfiguration('depth_scale'),
            'max_depth': LaunchConfiguration('max_depth'),
            'min_depth': LaunchConfiguration('min_depth'),
            'downsample_factor': LaunchConfiguration('downsample_factor'),
        }]
    )
    
    return LaunchDescription([
        depth_topic_arg,
        color_topic_arg,
        camera_info_topic_arg,
        pointcloud_topic_arg,
        frame_id_arg,
        depth_scale_arg,
        max_depth_arg,
        min_depth_arg,
        downsample_factor_arg,
        pointcloud_node,
    ])
