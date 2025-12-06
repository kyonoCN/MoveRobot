"""
MoveIt + URDF场景综合Launch文件

启动MoveIt demo并加载URDF场景作为障碍物
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 包路径
    demo_urdf_share = get_package_share_directory('demo_urdf')
    mmk2_fake_moveit_share = FindPackageShare('mmk2_fake_moveit_config')
    
    # URDF文件
    urdf_file = os.path.join(demo_urdf_share, 'urdf', 'scene.urdf')
    with open(urdf_file, 'r') as f:
        scene_description = f.read()
    
    # 参数
    frame_id_arg = DeclareLaunchArgument(
        'frame_id', default_value='base_link',
        description='场景参考坐标系'
    )
    
    # ========== 启动MoveIt Demo ==========
    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([mmk2_fake_moveit_share, 'launch', 'demo.launch.py'])
        ])
    )
    
    # ========== 静态TF: world -> base_link ==========
    # 将场景的world坐标系对齐到机器人的base_link
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'world'],
        output='screen'
    )
    
    # ========== 场景TF发布（用于RViz可视化） ==========
    scene_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='scene_state_publisher',
        namespace='demo_scene',
        output='screen',
        parameters=[{
            'robot_description': scene_description,
            'publish_frequency': 10.0,
        }]
    )
    
    # ========== URDF场景发布到MoveIt（延迟启动） ==========
    scene_publisher = TimerAction(
        period=5.0,  # 等待MoveIt初始化
        actions=[
            Node(
                package='demo_urdf',
                executable='publish_scene_to_moveit.py',
                name='urdf_scene_publisher',
                output='screen',
                parameters=[{
                    'urdf_file': urdf_file,
                    'frame_id': LaunchConfiguration('frame_id'),
                    'scene_name': 'demo_scene',
                    'publish_rate': 0.5,
                    'one_shot': False,
                }]
            )
        ]
    )
    
    return LaunchDescription([
        frame_id_arg,
        moveit_demo,
        static_tf,
        scene_state_publisher,
        scene_publisher,
    ])
