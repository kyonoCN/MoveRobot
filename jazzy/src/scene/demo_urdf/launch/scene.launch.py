"""
URDF场景发布Launch文件

功能：
1. 发布URDF场景的TF（可选，用于RViz可视化）
2. 将场景作为障碍物发布到MoveIt PlanningScene

使用方法：
ros2 launch demo_urdf scene.launch.py

与MoveIt配合使用：
1. 先启动MoveIt demo
2. 再启动此launch
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('demo_urdf')
    
    # URDF文件路径
    urdf_file = os.path.join(pkg_share, 'urdf', 'scene.urdf')
    
    # 读取URDF内容（用于robot_state_publisher）
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    # 声明参数
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='world',
        description='场景的参考坐标系（需要与机器人的世界坐标系一致）'
    )
    
    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='是否发布TF用于可视化'
    )
    
    # ========== 节点1: Robot State Publisher ==========
    # 发布场景的TF树，用于RViz可视化
    # 注意：使用不同的命名空间避免与机器人冲突
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='scene_state_publisher',
        namespace='demo_scene',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 10.0,
        }],
        remappings=[
            # 重映射话题避免冲突
            ('/demo_scene/robot_description', '/demo_scene/scene_description'),
        ]
    )
    
    # ========== 节点2: 静态TF发布 ==========
    # 发布world到机器人base_link的变换（如果需要对齐）
    # 这里假设场景的world与机器人的base_link对齐
    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'world'],
        output='screen'
    )
    
    # ========== 节点3: URDF场景发布到MoveIt ==========
    # 将URDF中的collision元素发布到MoveIt PlanningScene
    scene_publisher = Node(
        package='demo_urdf',
        executable='publish_scene_to_moveit.py',
        name='urdf_scene_publisher',
        output='screen',
        parameters=[{
            'urdf_file': urdf_file,
            'frame_id': LaunchConfiguration('frame_id'),
            'scene_name': 'demo_scene',
            'publish_rate': 0.5,
            'one_shot': False,  # 持续发布以确保MoveIt接收
        }]
    )
    
    return LaunchDescription([
        frame_id_arg,
        publish_tf_arg,
        robot_state_publisher,
        static_tf_publisher,
        scene_publisher,
    ])
