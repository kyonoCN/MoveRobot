"""
MoveIt + 工位场景综合Launch文件

启动MoveIt demo并加载工位(gongwei) URDF场景作为障碍物
参考demo_urdf实现
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
    ws_urdf_share = get_package_share_directory('ws_urdf')
    mmk2_fake_moveit_share = FindPackageShare('mmk2_fake_moveit_config')
    
    # URDF文件
    urdf_file = os.path.join(ws_urdf_share, 'urdf', 'gongwei.urdf')
    with open(urdf_file, 'r') as f:
        scene_description = f.read()
    
    # 参数
    frame_id_arg = DeclareLaunchArgument(
        'frame_id', default_value='base_link',
        description='场景参考坐标系'
    )
    
    mesh_scale_arg = DeclareLaunchArgument(
        'mesh_scale', default_value='1.0',
        description='Mesh缩放比例（STL已经是米单位）'
    )
    
    # ========== 启动MoveIt Demo ==========
    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([mmk2_fake_moveit_share, 'launch', 'demo.launch.py'])
        ])
    )
    
    # ========== 静态TF: base_link -> gongwei_base ==========
    # 将工位坐标系连接到机器人base_link，可通过参数调整位置
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gongwei_tf',
        arguments=[
            '1.3', '0', '0.82', '3.14159', '0', '0', 'base_link', 'gongwei_base'
        ],
        output='screen'
    )
    
    # ========== 场景TF发布（用于RViz可视化） ==========
    # 使用独立命名空间避免话题冲突
    scene_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='gongwei_state_publisher',
        namespace='gongwei',
        output='screen',
        parameters=[{
            'robot_description': scene_description,
            'publish_frequency': 10.0,
            'frame_prefix': '',  # 不添加前缀，使用URDF中定义的gongwei_base
        }]
    )
    
    # 位置参数
    position_x_arg = DeclareLaunchArgument(
        'position_x', default_value='0.3', description='工位X位置(米)'
    )
    position_y_arg = DeclareLaunchArgument(
        'position_y', default_value='0.0', description='工位Y位置(米)'
    )
    position_z_arg = DeclareLaunchArgument(
        'position_z', default_value='0.82', description='工位Z位置(米)'
    )
    
    # ========== 工位Mesh发布到MoveIt（延迟启动） ==========
    # 使用专门处理STL mesh的脚本
    scene_publisher = TimerAction(
        period=5.0,  # 等待MoveIt初始化
        actions=[
            Node(
                package='ws_urdf',
                executable='publish_ws_to_moveit.py',
                name='gongwei_mesh_publisher',
                output='screen',
                parameters=[{
                    'position_x': LaunchConfiguration('position_x'),
                    'position_y': LaunchConfiguration('position_y'),
                    'position_z': LaunchConfiguration('position_z'),
                    'frame_id': LaunchConfiguration('frame_id'),
                    'mesh_scale': LaunchConfiguration('mesh_scale'),
                }]
            )
        ]
    )
    
    return LaunchDescription([
        frame_id_arg,
        mesh_scale_arg,
        position_x_arg,
        position_y_arg,
        position_z_arg,
        moveit_demo,
        static_tf,
        scene_state_publisher,
        scene_publisher,
    ])
