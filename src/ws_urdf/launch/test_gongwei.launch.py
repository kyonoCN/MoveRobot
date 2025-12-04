"""
测试工位URDF显示

单独启动工位URDF，在RViz中查看是否正常显示
测试坐标系和mesh加载
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取包路径
    ws_urdf_share = get_package_share_directory('ws_urdf')
    
    # 读取URDF
    urdf_file = os.path.join(ws_urdf_share, 'urdf', 'gongwei.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    # 参数
    position_x_arg = DeclareLaunchArgument('position_x', default_value='0.0')
    position_y_arg = DeclareLaunchArgument('position_y', default_value='0.0')
    position_z_arg = DeclareLaunchArgument('position_z', default_value='0.0')
    mesh_scale_arg = DeclareLaunchArgument('mesh_scale', default_value='1.0')
    
    # ========== 1. Robot State Publisher (用于RViz可视化) ==========
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='gongwei_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 10.0,
        }]
    )
    
    # ========== 2. Joint State Publisher (发布关节状态) ==========
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='gongwei_joint_state_publisher',
        output='screen',
    )
    
    # ========== 3. 静态TF: world -> gongwei_base ==========
    # 工位坐标系直接连接到world
    static_tf_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_tf',
        arguments=[
            '--x', LaunchConfiguration('position_x'),
            '--y', LaunchConfiguration('position_y'),
            '--z', LaunchConfiguration('position_z'),
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'world',
            '--child-frame-id', 'gongwei_base'
        ],
        output='screen'
    )
    
    # ========== 4. RViz ==========
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(ws_urdf_share, 'config', 'gongwei.rviz')],
    )
    
    # ========== 5. 发布mesh到PlanningScene (延迟启动) ==========
    mesh_publisher = TimerAction(
        period=3.0,
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
                    'frame_id': 'world',  # 使用world坐标系
                    'mesh_scale': LaunchConfiguration('mesh_scale'),
                }]
            )
        ]
    )
    
    return LaunchDescription([
        position_x_arg,
        position_y_arg,
        position_z_arg,
        mesh_scale_arg,
        robot_state_publisher,
        joint_state_publisher,
        static_tf_world,
        rviz_node,
        mesh_publisher,
    ])
