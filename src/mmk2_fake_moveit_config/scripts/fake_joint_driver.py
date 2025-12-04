#!/usr/bin/env python3
"""
Fake Joint Driver for Virtual Robot
模拟轨迹执行并发布关节状态，用于离线 MoveIt 仿真

重要：控制器名称与真实机器人 (mmk2_moveit_config) 完全一致，
确保在虚拟机器人上开发的功能可以直接部署到真实机器人。

控制器名称映射：
  - left_arm_trajectory_controller  -> 左臂轨迹控制
  - right_arm_trajectory_controller -> 右臂轨迹控制
  - left_arm_eef_controller         -> 左夹爪控制
  - right_arm_eef_controller        -> 右夹爪控制
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from control_msgs.action import GripperCommand

import time
import threading


class FakeJointDriver(Node):
    def __init__(self):
        super().__init__('fake_joint_driver')
        
        self.callback_group = ReentrantCallbackGroup()
        
        # 定义所有关节
        self.all_joints = [
            # Left arm
            'left_arm_joint1', 'left_arm_joint2', 'left_arm_joint3',
            'left_arm_joint4', 'left_arm_joint5', 'left_arm_joint6',
            # Right arm
            'right_arm_joint1', 'right_arm_joint2', 'right_arm_joint3',
            'right_arm_joint4', 'right_arm_joint5', 'right_arm_joint6',
            # Grippers
            'left_arm_eef_gripper_joint', 'right_arm_eef_gripper_joint',
            # Head
            'head_yaw_joint', 'head_pitch_joint',
            # Spine
            'spine_joint',
        ]
        
        # 当前关节位置
        self.joint_positions = {name: 0.0 for name in self.all_joints}
        self.joint_velocities = {name: 0.0 for name in self.all_joints}
        
        # 锁
        self.lock = threading.Lock()
        
        # 关节状态发布器
        self.joint_state_pub = self.create_publisher(
            JointState, 
            'joint_states', 
            10
        )
        
        # 定时发布关节状态
        self.timer = self.create_timer(0.02, self.publish_joint_states)  # 50Hz
        
        # ============================================================
        # 创建轨迹 action servers
        # 控制器名称与真实机器人完全一致！
        # ============================================================
        self.left_arm_action = ActionServer(
            self,
            FollowJointTrajectory,
            'left_arm_trajectory_controller/follow_joint_trajectory',
            self.execute_trajectory_callback,
            callback_group=self.callback_group
        )
        
        self.right_arm_action = ActionServer(
            self,
            FollowJointTrajectory,
            'right_arm_trajectory_controller/follow_joint_trajectory',
            self.execute_trajectory_callback,
            callback_group=self.callback_group
        )
        
        # ============================================================
        # Gripper action servers
        # 控制器名称与真实机器人完全一致！
        # ============================================================
        self.left_gripper_action = ActionServer(
            self,
            GripperCommand,
            'left_arm_eef_controller/gripper_cmd',
            self.execute_gripper_callback,
            callback_group=self.callback_group
        )
        
        self.right_gripper_action = ActionServer(
            self,
            GripperCommand,
            'right_arm_eef_controller/gripper_cmd',
            self.execute_gripper_callback,
            callback_group=self.callback_group
        )
        
        self.get_logger().info('Fake Joint Driver started')
        self.get_logger().info('Controller names are consistent with real robot:')
        self.get_logger().info('  - left_arm_trajectory_controller')
        self.get_logger().info('  - right_arm_trajectory_controller')
        self.get_logger().info('  - left_arm_eef_controller')
        self.get_logger().info('  - right_arm_eef_controller')

    def publish_joint_states(self):
        """发布当前关节状态"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        with self.lock:
            msg.name = list(self.joint_positions.keys())
            msg.position = list(self.joint_positions.values())
            msg.velocity = list(self.joint_velocities.values())
            msg.effort = [0.0] * len(msg.name)
        
        self.joint_state_pub.publish(msg)

    def execute_trajectory_callback(self, goal_handle):
        """执行轨迹回调"""
        self.get_logger().info('Received trajectory request')
        
        trajectory = goal_handle.request.trajectory
        joint_names = trajectory.joint_names
        points = trajectory.points
        
        if len(points) == 0:
            goal_handle.succeed()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return result
        
        # 执行轨迹
        feedback = FollowJointTrajectory.Feedback()
        
        start_time = time.time()
        
        for i, point in enumerate(points):
            # 计算到达此点需要的时间
            target_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            
            # 等待到达目标时间
            while (time.time() - start_time) < target_time:
                time.sleep(0.01)
            
            # 更新关节位置
            with self.lock:
                for j, joint_name in enumerate(joint_names):
                    if joint_name in self.joint_positions:
                        self.joint_positions[joint_name] = point.positions[j]
                        if len(point.velocities) > j:
                            self.joint_velocities[joint_name] = point.velocities[j]
            
            # 发送反馈
            feedback.joint_names = joint_names
            feedback.actual.positions = point.positions
            feedback.desired.positions = point.positions
            goal_handle.publish_feedback(feedback)
        
        self.get_logger().info('Trajectory execution completed')
        
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    def execute_gripper_callback(self, goal_handle):
        """执行夹爪回调"""
        self.get_logger().info('Received gripper command')
        
        position = goal_handle.request.command.position
        
        goal_handle.succeed()
        result = GripperCommand.Result()
        result.position = position
        result.reached_goal = True
        return result


def main(args=None):
    rclpy.init(args=args)
    node = FakeJointDriver()
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
