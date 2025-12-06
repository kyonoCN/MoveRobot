import rclpy
import json
import time

from arm import DualArmController
from gripper import DualGripperController
from states import RobotJointStates


class MotionOrchestrator:
    """
    动作编排器类，用于解析和执行JSON格式的动作序列
    支持全身控制：双臂和双夹爪
    支持读取原始数据格式（含key_point字段）并转换为关键信息格式进行回放
    """
    
    def __init__(self, time_init=2.0, time_step=1.0):
        """
        初始化动作编排器
        
        Args:
            time_init (float): 初始时间偏移（秒）
            time_step (float): 每个路点之间的时间间隔（秒）
        """
        self.time_init = time_init
        self.time_step = time_step
        self.arm = DualArmController()
        self.gripper = DualGripperController()
        # self.states = RobotJointStates()
    
    def convert_raw_to_keyinfo(self, raw_data_list):
        """
        将原始数据格式转换为关键信息格式
        只处理key_point=True的元素，根据key_action字段提取对应的关键数值
        
        追加规则：
        - left_gripper, right_gripper, pause: 只保留1个值，连续同类型时替换旧值
        - left_arm, right_arm: 追加到现有列表，连续同类型时追加
        - 如果新元素的key_action与最后一个元素类型不同，新建一个字典
        
        Args:
            raw_data_list (list): 原始数据列表
            
        Returns:
            list: 关键信息格式的动作序列
        """
        # 筛选key_point=True的元素
        key_points = [item for item in raw_data_list if item.get('key_point', False)]
        
        if not key_points:
            print("警告: 没有找到key_point=True的元素")
            return []
        
        print(f"找到 {len(key_points)} 个关键点")
        
        # 构建关键信息格式的动作序列
        actions = []
        
        for point in key_points:
            key_action = point.get('key_action', '')
            
            if not key_action:
                print(f"警告: 元素缺少key_action字段，跳过")
                continue
            
            # 根据key_action提取对应的关键数值
            if key_action == 'left_gripper':
                value = point.get('left_gripper', [0.0])[0] if point.get('left_gripper') else 0.0
                self._append_single_value(actions, 'left_gripper', [value])
                
            elif key_action == 'right_gripper':
                value = point.get('right_gripper', [0.0])[0] if point.get('right_gripper') else 0.0
                self._append_single_value(actions, 'right_gripper', [value])
                
            elif key_action == 'pause':
                value = point.get('pause', 2.0)
                self._append_single_value(actions, 'pause', [value])
                
            elif key_action == 'left_arm':
                value = point.get('left_arm', [])
                if value:
                    self._append_array_value(actions, 'left_arm', value)
                    
            elif key_action == 'right_arm':
                value = point.get('right_arm', [])
                if value:
                    self._append_array_value(actions, 'right_arm', value)
        
        return actions
    
    def _append_single_value(self, actions, key, value):
        """
        追加单值类型（left_gripper, right_gripper, pause）
        如果最后一个元素是同类型，则替换；否则新建
        
        Args:
            actions (list): 动作序列列表
            key (str): 动作类型键
            value: 要追加的值
        """
        if actions and key in actions[-1]:
            # 同类型，替换值
            actions[-1][key] = value
        else:
            # 不同类型，新建字典
            actions.append({key: value})
    
    def _append_array_value(self, actions, key, value):
        """
        追加数组类型（left_arm, right_arm）
        如果最后一个元素是同类型，则追加到列表末尾；否则新建
        
        Args:
            actions (list): 动作序列列表
            key (str): 动作类型键
            value: 要追加的值（单个路点数组）
        """
        if actions and key in actions[-1]:
            # 同类型，追加到列表末尾
            actions[-1][key].append(value)
        else:
            # 不同类型，新建字典，值是包含该路点的列表
            actions.append({key: [value]})
        
    def execute_sequence(self, file_path):
        """
        执行给定JSON文件中的动作序列
        支持原始数据格式（含key_point字段）和关键信息格式
        
        Args:
            file_path (str): JSON文件路径
        """
        with open(file_path, 'r') as f:
            data = json.load(f)
        
        # 检测数据格式：原始格式或关键信息格式
        if data and isinstance(data[0], dict) and 'key_point' in data[0]:
            # 原始数据格式，需要转换
            print("检测到原始数据格式，正在转换为关键信息格式...")
            actions = self.convert_raw_to_keyinfo(data)
        else:
            # 已经是关键信息格式
            print("检测到关键信息格式，直接执行...")
            actions = data
        
        if not actions:
            print("没有可执行的动作序列")
            return
        
        print("动作序列：")
        print(actions)
        print(f"将执行 {len(actions)} 个动作")
        
        # 初始化轨迹点列表
        left_waypoints = []
        left_times = []
        right_waypoints = []
        right_times = []
        left_init = True
        right_init = True

        # 处理动作序列
        for i, action in enumerate(actions):
            if "pause" in action:
                # 暂停指定时间
                pause_duration = action["pause"][0]
                time.sleep(pause_duration)

            elif "left_gripper" in action:
                # 控制左夹爪
                gripper_position = action["left_gripper"][0]
                self.gripper.set_left(gripper_position)
                
            elif "right_gripper" in action:
                # 控制右夹爪
                gripper_position = action["right_gripper"][0]
                self.gripper.set_right(gripper_position)
                
            elif "left_arm" in action:
                # 添加左臂路点
                waypoints = action["left_arm"]
                left_waypoints.extend(waypoints)
                
                # 计算时间戳
                left_times = [(self.time_init + self.time_step * j) for j in range(len(waypoints))]

                # 执行左臂轨迹
                if left_waypoints and left_times:
                    self.arm.send_left(left_waypoints, left_times)
                    # print(left_waypoints, left_times)
                    
                # 重置轨迹
                left_waypoints = []
                left_times = []
                
            elif "right_arm" in action:
                # 添加右臂路点
                waypoints = action["right_arm"]
                right_waypoints.extend(waypoints)
                
                # 计算时间戳
                right_times = [self.time_init + self.time_step * j for j in range(len(waypoints))]

                # 执行右臂轨迹
                if right_waypoints and right_times:
                    self.arm.send_right(right_waypoints, right_times)
                    
                # 重置轨迹
                right_waypoints = []
                right_times = []
    
    # def get_current_states(self):
    #     """
    #     获取机器人当前全身状态
        
    #     Returns:
    #         dict: 包含所有关节状态的字典
    #     """
    #     return self.states.get_states()
        
    def destroy(self):
        """
        销毁所有节点
        """
        self.arm.destroy_node()
        self.gripper.destroy_node()
        # self.states.destroy_node()


def orchestrate(file_path):
    """
    编排函数，读取JSON文件并执行其中定义的动作序列
    
    Args:
        file_path (str): JSON文件路径
    """
    rclpy.init()
    
    try:
        orchestrator = MotionOrchestrator()
        orchestrator.execute_sequence(file_path)
        orchestrator.destroy()
    except Exception as e:
        print(f"执行过程中出现错误: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    # 使用编排函数执行JSON中定义的动作
    json_file = '/ros2_ws/src/process/data/1222.json'
    # json_file = '/ros2_ws/src/process/data/2.json'
    orchestrate(json_file)
