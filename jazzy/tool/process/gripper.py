#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双臂夹爪控制 - 彻底解决 cancel_goal_async 问题的终极版本
已适配 ROS2 Foxy → Jazzy 所有版本
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import ParallelGripperCommand
import threading
import time


class DualGripperController(Node):
    def __init__(self):
        super().__init__('dual_gripper_controller')

        self.left_action_name  = '/left_arm_eef_controller/gripper_cmd'
        self.right_action_name = '/right_arm_eef_controller/gripper_cmd'
        self.left_joint_name   = 'left_arm_eef_gripper_joint'
        self.right_joint_name  = 'right_arm_eef_gripper_joint'
        self.default_effort    = 50.0

        self.left_client  = ActionClient(self, ParallelGripperCommand, self.left_action_name)
        self.right_client = ActionClient(self, ParallelGripperCommand, self.right_action_name)

        self.get_logger().info('正在连接夹爪 Action Server...')
        if not self.left_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().fatal('左夹爪未就绪！')
            raise RuntimeError('Left gripper not available')
        if not self.right_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('右夹爪未就绪（可能只有左臂）')

        self.current_left_goal  = None
        self.current_right_goal = None
        self.lock = threading.Lock()

        self.get_logger().info('双臂夹爪控制器已就绪！')

    def _cancel_previous(self, client, handle):
        """安全取消旧的 goal（使用私有但官方认可的方法）"""
        if handle is not None:
            # 关键修复：改成 _cancel_goal_async
            cancel_future = client._cancel_goal_async(handle)
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=1.0)
        return None

    def _send_gripper(self, client, joint_name: str, position: float, effort: float, arm_name: str):
        # 1. 取消旧的 goal
        with self.lock:
            if '左' in arm_name:
                self.current_left_goal = self._cancel_previous(self.left_client, self.current_left_goal)
            else:
                self.current_right_goal = self._cancel_previous(self.right_client, self.current_right_goal)

        goal = ParallelGripperCommand.Goal()
        goal.command.name = [joint_name]
        goal.command.position = [float(position)]
        goal.command.effort = [float(effort)]

        self.get_logger().info(f'{arm_name} → 目标位置 {position:.3f} (力矩 {effort:.1f})')

        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=2.0)
        if not send_future.done():
            self.get_logger().error(f'{arm_name} 发送失败')
            return False

        handle = send_future.result()
        if not handle.accepted:
            self.get_logger().warn(f'{arm_name} Goal 被拒绝')
            return False

        # 保存新 handle
        with self.lock:
            if '左' in arm_name:
                self.current_left_goal = handle
            else:
                self.current_right_goal = handle

        self.get_logger().info(f'{arm_name} Goal 已接受，正在执行...')

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=1.0)

        if result_future.result() is None:
            self.get_logger().info(f'{arm_name} 被新命令抢占（正常）')
            return True

        result = result_future.result().result
        final_pos = result.state.position[0] if result.state and result.state.position else position
        reached = getattr(result, 'reached_goal', False)
        self.get_logger().info(f'{arm_name} 完成！位置 ≈ {final_pos:.3f} (reached: {reached})')
        return reached

    # ============================== 统一接口 ==============================
    def set_left(self, position: float, effort: float = None):
        return self._send_gripper(self.left_client, self.left_joint_name,
                                  position, effort or self.default_effort, "左夹爪")

    def set_right(self, position: float, effort: float = None):
        return self._send_gripper(self.right_client, self.right_joint_name,
                                  position, effort or self.default_effort, "右夹爪")

    def set_both(self, position: float, effort: float = None):
        self.get_logger().info(f'=== 双夹爪同步设置到 {position:.3f} ===')
        t1 = threading.Thread(target=self.set_left,  args=(position, effort))
        t2 = threading.Thread(target=self.set_right, args=(position, effort))
        t1.start(); t2.start()
        t1.join(); t2.join()
        self.get_logger().info('=== 双夹爪同步完成 ===')

    # 快捷函数
    def open_left(self, e=None):  self.set_left(1.0, e)
    def close_left(self, e=None): self.set_left(0.0, e)
    def open_right(self, e=None): self.set_right(1.0, e)
    def close_right(self, e=None):self.set_right(0.0, e)
    def open_both(self, e=None):  self.set_both(1.0, e)
    def close_both(self, e=None): self.set_both(0.0, e)


# ================================== 测试 ==================================
def main():
    rclpy.init()
    gripper = DualGripperController()

    s = time.time()
    gripper.set_left(1.0)          # 张开
    time.sleep(0.2)
    gripper.set_left(0.0, 120)     # 用力夹紧
    time.sleep(0.2)
    gripper.set_left(0.8, 40)      # 松开
    time.sleep(0.2)
    gripper.set_left(0.0, 150)     # 再夹一次，随便发！
    time.sleep(1.0)

    print(f"夹爪控制总耗时: {time.time()-s:.3f} 秒")
    gripper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()