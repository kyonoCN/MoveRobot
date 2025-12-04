"""
带工位场景的MoveIt2 Demo Launch文件
启动机器人MoveIt demo，同时加载工位(gongwei) mesh作为障碍物用于避障规划
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 获取包路径
    mmk2_pkg = FindPackageShare('mmk2_moveit_config')
    
    # 工位位置参数
    position_x_arg = DeclareLaunchArgument(
        'position_x',
        default_value='0.5',
        description='工位X位置(米)'
    )
    position_y_arg = DeclareLaunchArgument(
        'position_y',
        default_value='0.0',
        description='工位Y位置(米)'
    )
    position_z_arg = DeclareLaunchArgument(
        'position_z',
        default_value='0.0',
        description='工位Z位置(米)'
    )
    
    # 工位姿态参数（四元数）
    orientation_x_arg = DeclareLaunchArgument(
        'orientation_x',
        default_value='0.0',
        description='工位姿态 quaternion x'
    )
    orientation_y_arg = DeclareLaunchArgument(
        'orientation_y',
        default_value='0.0',
        description='工位姿态 quaternion y'
    )
    orientation_z_arg = DeclareLaunchArgument(
        'orientation_z',
        default_value='0.0',
        description='工位姿态 quaternion z'
    )
    orientation_w_arg = DeclareLaunchArgument(
        'orientation_w',
        default_value='1.0',
        description='工位姿态 quaternion w'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='base_link',
        description='工位参考坐标系（需与机器人坐标系一致）'
    )
    
    mesh_scale_arg = DeclareLaunchArgument(
        'mesh_scale',
        default_value='0.001',
        description='Mesh缩放比例(STL通常是mm需要转为m)'
    )
    
    # 包含原始demo launch（启动机器人、MoveIt、RViz）
    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([mmk2_pkg, 'launch', 'demo.launch.py'])
        ])
    )
    
    # 工位障碍物发布节点（延迟启动，等待MoveIt完全启动）
    workstation_publisher = TimerAction(
        period=5.0,  # 延迟5秒启动，确保MoveIt已准备好
        actions=[
            Node(
                package='ws_urdf',
                executable='publish_ws_to_moveit.py',
                name='workstation_publisher',
                output='screen',
                parameters=[{
                    'position_x': LaunchConfiguration('position_x'),
                    'position_y': LaunchConfiguration('position_y'),
                    'position_z': LaunchConfiguration('position_z'),
                    'orientation_x': LaunchConfiguration('orientation_x'),
                    'orientation_y': LaunchConfiguration('orientation_y'),
                    'orientation_z': LaunchConfiguration('orientation_z'),
                    'orientation_w': LaunchConfiguration('orientation_w'),
                    'frame_id': LaunchConfiguration('frame_id'),
                    'mesh_scale': LaunchConfiguration('mesh_scale'),
                }]
            )
        ]
    )
    
    return LaunchDescription([
        # 参数声明
        position_x_arg,
        position_y_arg,
        position_z_arg,
        orientation_x_arg,
        orientation_y_arg,
        orientation_z_arg,
        orientation_w_arg,
        frame_id_arg,
        mesh_scale_arg,
        # 启动
        demo_launch,
        workstation_publisher,
    ])
