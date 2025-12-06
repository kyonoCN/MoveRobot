"""
简单的机器人可视化 Launch 文件

功能:
- 显示完整的 mmk2 机器人模型 (主干 + 双臂 + 夹爪)
- 显示 TF 坐标系
- 不需要真实硬件

使用方法:
  cd /ros2_ws
  source install/setup.bash
  ros2 launch ws_urdf view_robot_tf.launch.py

可选参数:
  use_gui:=true   - 启动关节滑块GUI来手动控制关节
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ============================================================
    # 参数声明
    # ============================================================
    use_gui_arg = DeclareLaunchArgument(
        "use_gui",
        default_value="false",
        description="是否启动 joint_state_publisher_gui（用于手动拖动关节）"
    )

    # ============================================================
    # 获取包路径
    # ============================================================
    ws_urdf_share = get_package_share_directory("ws_urdf")
    mmk2_urdf_share = get_package_share_directory("mmk2_urdf")
    
    # URDF 文件路径
    urdf_file = os.path.join(mmk2_urdf_share, "urdf", "mmk2.urdf")
    
    # RViz 配置文件
    rviz_config = os.path.join(ws_urdf_share, "rviz", "view_robot_tf.rviz")
    
    # 读取 URDF 内容
    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()

    # ============================================================
    # 1. Static TF: world -> base_link
    # ============================================================
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    # ============================================================
    # 2. Robot State Publisher - 发布机器人 TF
    # ============================================================
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_content}],
    )

    # ============================================================
    # 3. Joint State Publisher - 发布关节状态
    # ============================================================
    # 不带GUI的版本 - 所有关节保持初始位置
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("use_gui")),
    )

    # 带GUI的版本 - 可以手动拖动关节
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_gui")),
    )

    # ============================================================
    # 4. RViz2 - 可视化界面
    # ============================================================
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"robot_description": robot_description_content}],
    )

    # ============================================================
    # 返回 Launch Description
    # ============================================================
    return LaunchDescription([
        use_gui_arg,
        static_tf_node,
        robot_state_publisher,
        joint_state_publisher,
        joint_state_publisher_gui,
        rviz_node,
    ])
