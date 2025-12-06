#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双臂轨迹控制终极版 - 支持同步、异步、抢占、实时反馈
作者：你的名字
日期：2025
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from typing import List, Optional, Dict, Union
import threading
import time


class DualArmController(Node):
    def __init__(self):
        super().__init__('dual_arm_controller')

        # 左右臂关节名（请根据你的机器人实际修改！）
        self.left_joints = [
            'left_arm_joint1', 'left_arm_joint2', 'left_arm_joint3',
            'left_arm_joint4', 'left_arm_joint5', 'left_arm_joint6'
        ]
        self.right_joints = [
            'right_arm_joint1', 'right_arm_joint2', 'right_arm_joint3',
            'right_arm_joint4', 'right_arm_joint5', 'right_arm_joint6'
        ]

        # Action Clients
        self.left_client = ActionClient(self, FollowJointTrajectory,
                                       '/left_arm_trajectory_controller/follow_joint_trajectory')
        self.right_client = ActionClient(self, FollowJointTrajectory,
                                        '/right_arm_trajectory_controller/follow_joint_trajectory')

        # 等待两个 action server 都起来（最多等15秒）
        self.get_logger().info('Waiting for both arm action servers...')
        if not self.left_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().fatal('Left arm action server not available!')
        if not self.right_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().fatal('Right arm action server not available!')
        self.get_logger().info('Both arms ready!')

        # 当前 goal handle（用于抢占）
        self.left_goal_handle = None
        self.right_goal_handle = None
        self.lock = threading.Lock()

    def _build_trajectory(self, joint_names: List[str], waypoints: List[List[float]], times: List[float]) -> JointTrajectory:
        traj = JointTrajectory()
        traj.joint_names = joint_names
        for pos, t in zip(waypoints, times):
            point = JointTrajectoryPoint()
            point.positions = pos.copy()
            point.time_from_start = Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))
            traj.points.append(point)
        return traj

    def send_left(self,
                  waypoints: List[List[float]],
                  times: List[float],
                  wait: bool = True,
                  feedback: bool = True) -> bool:
        """发送左臂轨迹"""
        return self._send_arm('left', waypoints, times, wait, feedback)

    def send_right(self,
                   waypoints: List[List[float]],
                   times: List[float],
                   wait: bool = True,
                   feedback: bool = True) -> bool:
        """发送右臂轨迹"""
        return self._send_arm('right', waypoints, times, wait, feedback)

    def send_both_sync(self,
                       left_waypoints: List[List[float]], left_times: List[float],
                       right_waypoints: List[List[float]], right_times: List[float],
                       feedback: bool = True) -> bool:
        """双臂同步执行（同时开始，同时结束）"""
        self.get_logger().info('=== 双臂同步运动开始 ===')
        t1 = threading.Thread(target=self.send_left, args=(left_waypoints, left_times, True, feedback))
        t2 = threading.Thread(target=self.send_right, args=(right_waypoints, right_times, True, feedback))
        t1.start(); t2.start()
        t1.join(); t2.join()
        self.get_logger().info('=== 双臂同步运动完成 ===')
        return True

    def _send_arm(self, arm: str, waypoints: List[List[float]], times: List[float],
                  wait: bool, feedback: bool) -> bool:
        client = self.left_client if arm == 'left' else self.right_client
        joints = self.left_joints if arm == 'left' else self.right_joints
        arm_name = "左臂" if arm == 'left' else "右臂"

        traj = self._build_trajectory(joints, waypoints, times)
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj

        with self.lock:
            if arm == 'left':
                self.left_goal_handle = None
            else:
                self.right_goal_handle = None

        self.get_logger().info(f'→ {arm_name} 发送 {len(waypoints)} 个路点轨迹')

        send_goal_future = client.send_goal_async(
            goal_msg,
            feedback_callback=self._make_feedback_cb(arm_name) if feedback else None
        )
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=10.0)

        if not send_goal_future.done():
            self.get_logger().error(f'{arm_name} Goal 发送失败')
            return False

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'{arm_name} Goal 被拒绝')
            return False

        with self.lock:
            if arm == 'left':
                self.left_goal_handle = goal_handle
            else:
                self.right_goal_handle = goal_handle

        self.get_logger().info(f'{arm_name} Goal 被接受，正在执行...')

        if not wait:
            return True

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        if result.error_code == 0:
            self.get_logger().info(f'{arm_name} 轨迹执行成功！')
        else:
            self.get_logger().error(f'{arm_name} 执行失败: {result.error_string}')
        return result.error_code == 0

    def _make_feedback_cb(self, arm_name: str):
        def callback(feedback_msg):
            f = feedback_msg.feedback
            err = f.error.positions if f.error.positions else [0]*6
            self.get_logger().info(
                f'{arm_name} | t={f.actual.time_from_start.sec + f.actual.time_from_start.nanosec/1e9:.2f}s '
                f'| pos0={f.actual.positions[0]:.3f} | err_max={max(map(abs,err)):.4f}'
            )
        return callback

    def stop_left(self):
        self._cancel_goal(self.left_client, self.left_goal_handle, "左臂")

    def stop_right(self):
        self._cancel_goal(self.right_client, self.right_goal_handle, "右臂")

    def stop_both(self):
        self.get_logger().warn("紧急停止双臂！")
        self.stop_left()
        self.stop_right()

    def _cancel_goal(self, client, handle, name):
        if handle is not None:
            cancel_future = client.cancel_goal_async(handle)
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
            self.get_logger().warn(f"{name} 已发送取消指令（平滑刹车）")
        else:
            self.get_logger().info(f"{name} 无运行中任务")


# ============================== 使用示例 ==============================
def main():
    rclpy.init()
    arm = DualArmController()

    # 示例1：左臂单独动
    left_wp = [
        [0.519, -0.139, 0.511, -0.531, 0.248, 0.369],
        [0.719, -0.339, 0.711, -0.731, 0.448, 0.569],
        [0.419, -0.139, 0.511, -0.531, 0.248, 0.369],
    ]
    left_t = [0.0, 5.0, 12.0]

    # 示例2：右臂单独动（镜像）
    right_wp = [
        [-0.519, 0.139, 0.511, 0.531, 0.248, -0.369],   # 注意右臂坐标通常是镜像的！
        [-0.719, 0.339, 0.711, 0.731, 0.448, -0.569],
        [-0.419, 0.139, 0.511, 0.531, 0.248, -0.369],
    ]
    right_t = [0.0, 5.0, 12.0]

    # 选择一种方式运行：

    # 1. 左臂单独动
    arm.send_left(left_wp, left_t)

    # 2. 右臂单独动
    # arm.send_right(right_wp, right_t)

    # 3. 双臂同步镜像运动（最帅！）
    # arm.send_both_sync(left_wp, left_t, right_wp, right_t)

    # 4. 异步：左臂先动，右臂晚2秒再动
    # arm.send_left(left_wp, left_t, wait=False)
    # time.sleep(2.0)
    # arm.send_right(right_wp, right_t)

    # 运行中想急停：Ctrl+C 或调用
    # arm.stop_both()

    arm.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()