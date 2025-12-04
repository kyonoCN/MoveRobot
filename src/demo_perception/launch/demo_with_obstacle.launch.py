"""
带障碍物的MoveIt2 Demo Launch文件
在原demo基础上添加了障碍物节点，会在场景中添加一个Box用于避障规划
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 获取包路径
    pkg_share = FindPackageShare('mmk2_moveit_config')
    
    # 声明参数
    box_x_arg = DeclareLaunchArgument('box_x', default_value='0.5', description='障碍物X位置')
    box_y_arg = DeclareLaunchArgument('box_y', default_value='0.0', description='障碍物Y位置')
    box_z_arg = DeclareLaunchArgument('box_z', default_value='0.5', description='障碍物Z位置')
    box_size_x_arg = DeclareLaunchArgument('box_size_x', default_value='0.1', description='障碍物X尺寸')
    box_size_y_arg = DeclareLaunchArgument('box_size_y', default_value='0.2', description='障碍物Y尺寸')
    box_size_z_arg = DeclareLaunchArgument('box_size_z', default_value='1.6', description='障碍物Z尺寸')
    frame_id_arg = DeclareLaunchArgument('frame_id', default_value='base_link', description='参考坐标系')
    
    # 包含原始demo launch
    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_share, 'launch', 'demo.launch.py'])
        ])
    )
    
    # 障碍物添加节点
    add_obstacle_node = Node(
        package='mmk2_moveit_config',
        executable='add_obstacle_node.py',
        name='add_obstacle_node',
        output='screen',
        parameters=[{
            'box_x': LaunchConfiguration('box_x'),
            'box_y': LaunchConfiguration('box_y'),
            'box_z': LaunchConfiguration('box_z'),
            'box_size_x': LaunchConfiguration('box_size_x'),
            'box_size_y': LaunchConfiguration('box_size_y'),
            'box_size_z': LaunchConfiguration('box_size_z'),
            'frame_id': LaunchConfiguration('frame_id'),
            'obstacle_id': 'obstacle_box',
        }]
    )
    
    return LaunchDescription([
        box_x_arg,
        box_y_arg,
        box_z_arg,
        box_size_x_arg,
        box_size_y_arg,
        box_size_z_arg,
        frame_id_arg,
        demo_launch,
        add_obstacle_node,
    ])
