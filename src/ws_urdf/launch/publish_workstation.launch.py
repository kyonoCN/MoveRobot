"""
工位障碍物发布Launch文件
将工位mesh发布到MoveIt planning scene作为障碍物
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 声明参数
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
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='world',
        description='参考坐标系'
    )
    mesh_scale_arg = DeclareLaunchArgument(
        'mesh_scale',
        default_value='0.001',
        description='Mesh缩放比例(STL通常是mm需要转为m)'
    )
    
    # 工位发布节点
    workstation_publisher = Node(
        package='ws_urdf',
        executable='publish_ws_to_moveit.py',
        name='workstation_publisher',
        output='screen',
        parameters=[{
            'position_x': LaunchConfiguration('position_x'),
            'position_y': LaunchConfiguration('position_y'),
            'position_z': LaunchConfiguration('position_z'),
            'frame_id': LaunchConfiguration('frame_id'),
            'mesh_scale': LaunchConfiguration('mesh_scale'),
        }]
    )
    
    return LaunchDescription([
        position_x_arg,
        position_y_arg,
        position_z_arg,
        frame_id_arg,
        mesh_scale_arg,
        workstation_publisher,
    ])
