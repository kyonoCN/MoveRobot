"""
虚拟机器人 MoveIt2 Demo Launch 文件
无需真实机器人即可进行 Plan 和可视化控制

特点:
- 完全独立的虚拟机器人系统，不影响真实机器人配置
- 控制器名称与真实机器人完全一致，确保无缝部署
- 不启动 ros2_control，不需要真实硬件连接
- 使用 fake_joint_driver 发布关节状态，实现实时可视化
- 可以在 RViz 中拖拽末端执行器并执行 Plan & Execute
- Execute 后虚拟机器人会移动到规划位置

使用方法:
  ros2 launch mmk2_fake_moveit_config demo.launch.py

可选参数:
  use_rviz:=true/false  - 是否启动 RViz (默认 true)
  use_gui:=true/false   - 是否启动关节滑块GUI (默认 false)
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # ============================================================
    # 参数声明
    # ============================================================
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="是否启动 RViz"
    )
    
    use_gui_arg = DeclareLaunchArgument(
        "use_gui",
        default_value="false",
        description="是否启动 joint_state_publisher_gui（用于手动拖动关节）"
    )

    # ============================================================
    # 获取包路径
    # ============================================================
    # 使用真实机器人包的共享配置（SRDF, kinematics, joint_limits等）
    mmk2_moveit_share = get_package_share_directory("mmk2_moveit_config")
    # 虚拟机器人包（本包）的特定配置
    fake_moveit_share = get_package_share_directory("mmk2_fake_moveit_config")
    mmk2_urdf_share = get_package_share_directory("mmk2_urdf")
    
    # URDF 文件路径
    urdf_file = os.path.join(mmk2_urdf_share, "urdf", "mmk2.urdf")
    
    # 读取 URDF 内容
    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()

    # ============================================================
    # 构建 MoveIt 配置
    # 使用真实机器人包的共享配置，但使用本包的控制器配置
    # ============================================================
    moveit_config = (
        MoveItConfigsBuilder("mmk2", package_name="mmk2_moveit_config")
        .robot_description(file_path=urdf_file)
        .robot_description_semantic(file_path="config/mmk2.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        # 使用虚拟机器人包的控制器配置（控制器名称与真实机器人一致）
        .trajectory_execution(
            file_path=os.path.join(fake_moveit_share, "config", "moveit_controllers.yaml")
        )
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner", "stomp"])
        .to_moveit_configs()
    )

    # RViz 配置文件（使用真实机器人包的配置）
    rviz_config = os.path.join(mmk2_moveit_share, "config", "moveit.rviz")

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
    # 2. Robot State Publisher - 发布机器人 TF 和 robot_description topic
    # ============================================================
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description_content},
            {"publish_frequency": 50.0},
        ],
    )

    # ============================================================
    # 3. Fake Joint Driver - 模拟轨迹执行并发布关节状态
    # 控制器名称与真实机器人完全一致
    # ============================================================
    fake_joint_driver = Node(
        package="mmk2_fake_moveit_config",
        executable="fake_joint_driver.py",
        name="fake_joint_driver",
        output="screen",
    )

    # Joint State Publisher GUI（可选，用于手动控制关节）
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_gui")),
    )

    # ============================================================
    # 4. MoveIt move_group Node - 核心规划节点
    # ============================================================
    move_group_params = moveit_config.to_dict()
    
    # 关键配置
    move_group_params.update({
        "use_sim_time": False,
        "publish_robot_description_semantic": True,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        # 轨迹执行配置（宽松设置，适合仿真）
        "trajectory_execution.allowed_execution_duration_scaling": 1.5,
        "trajectory_execution.allowed_goal_duration_margin": 1.0,
        "trajectory_execution.allowed_start_tolerance": 0.05,
        "trajectory_execution.execution_duration_monitoring": False,
    })

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[move_group_params],
    )

    # ============================================================
    # 5. RViz2 - 可视化界面
    # ============================================================
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    # ============================================================
    # 返回 Launch Description
    # ============================================================
    return LaunchDescription([
        # 参数
        use_rviz_arg,
        use_gui_arg,
        # 节点（按启动顺序）
        static_tf_node,
        robot_state_publisher,
        fake_joint_driver,
        joint_state_publisher_gui,
        move_group_node,
        rviz_node,
    ])
