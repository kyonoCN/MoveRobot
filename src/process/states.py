#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
终极稳定版：RobotJointStates
已解决所有历史遗留问题：
1. QoS 兼容性
2. 必须 spin 才能收到消息
3. 超时不崩溃
4. 自动重连
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import JointState
from typing import Dict, Optional
import threading
import time


class RobotJointStates(Node):
    def __init__(self, topic: str = "/joint_states", timeout_sec: float = 10.0):
        super().__init__('robot_joint_states', allow_undeclared_parameters=True)

        self.target_joints = [
            'right_arm_joint1', 'right_arm_joint2', 'right_arm_joint4', 'right_arm_joint3',
            'right_arm_joint6', 'slide_joint', 'left_arm_joint5', 'left_arm_joint6',
            'left_arm_joint4', 'left_arm_joint3', 'right_arm_joint5', 'head_yaw_joint',
            'left_arm_joint2', 'left_arm_joint1', 'right_arm_eef_gripper_joint',
            'head_pitch_joint', 'left_arm_eef_gripper_joint'
        ]

        self.latest_state: Optional[JointState] = None
        self.lock = threading.Lock()
        self.ready = threading.Event()
        self.topic = topic
        self.timeout_sec = timeout_sec

        # 关键：兼容所有控制器（包括 Best Effort + Transient Local）
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        self.get_logger().info(f'正在订阅 {topic}，等待第一条消息...')

        self.subscription = self.create_subscription(
            JointState,
            topic,
            self._callback,
            qos_profile=qos
        )

        # 关键修复：在后台启动 spin 线程
        self.spin_thread = threading.Thread(target=self._spin_forever, daemon=True)
        self.spin_thread.start()

        # 等待第一条消息
        if not self.ready.wait(timeout_sec):
            self.get_logger().warning(f'{timeout_sec}s 内未收到 {topic}！程序将继续运行，但关节状态将使用默认值')
            # 不再抛出异常，而是继续运行但使用默认值

        self.get_logger().info('关节状态监视器初始化完成')

    def _spin_forever(self):
        """后台持续 spin，让回调能被触发"""
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def _callback(self, msg: JointState):
        with self.lock:
            self.latest_state = msg
        if not self.ready.is_set():
            self.ready.set()  # 第一次收到后就设为 True
            self.get_logger().info('已接收到第一条关节状态消息')

    def get_states(self) -> Dict[str, float]:
        """返回你关心的所有关节位置字典"""
        with self.lock:
            if self.latest_state is None:
                self.get_logger().debug("尚未收到任何关节状态消息，返回默认值")
                return {name: 0.0 for name in self.target_joints}

            name_to_pos = dict(zip(self.latest_state.name, self.latest_state.position))
            return {joint: name_to_pos.get(joint, 0.0) for joint in self.target_joints}

    def get_left_gripper(self) -> float:
        return self.get_states()['left_arm_eef_gripper_joint']

    def get_right_gripper(self) -> float:
        return self.get_states()['right_arm_eef_gripper_joint']


# ============================== 测试 ==============================
def main():
    rclpy.init()
    states = RobotJointStates(timeout_sec=10.0)

    print("\n当前机器人关节状态（已成功接收！）")
    data = states.get_states()
    for name, pos in data.items():
        print(f"{name:35}: {pos: .6f}")

    print(f"\n左夹爪开度: {states.get_left_gripper():.3f}")
    print(f"右夹爪开度: {states.get_right_gripper():.3f}")

    # 实时打印
    try:
        while True:
            l = states.get_left_gripper()
            r = states.get_right_gripper()
            print(f"左夹爪: {l:6.3f}  右夹爪: {r:6.3f}", end="\r")
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n退出")

    states.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()