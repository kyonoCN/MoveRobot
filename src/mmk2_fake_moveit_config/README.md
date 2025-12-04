# mmk2_fake_moveit_config

虚拟机器人 MoveIt2 配置包，用于离线仿真和开发。

## 特点

- **控制器名称与真实机器人完全一致**：确保在虚拟机器人上开发的功能可以直接部署到真实机器人
- 无需真实硬件连接
- 使用 `fake_joint_driver` 模拟轨迹执行
- 可在 RViz 中进行 Plan & Execute 操作

## 控制器名称映射

| 控制器名称 | 功能 |
|-----------|------|
| `left_arm_trajectory_controller` | 左臂轨迹控制 |
| `right_arm_trajectory_controller` | 右臂轨迹控制 |
| `left_arm_eef_controller` | 左夹爪控制 |
| `right_arm_eef_controller` | 右夹爪控制 |

## 使用方法

### 启动虚拟机器人
```bash
ros2 launch mmk2_fake_moveit_config demo.launch.py
```

### 可选参数
```bash
# 启动时打开关节滑块GUI
ros2 launch mmk2_fake_moveit_config demo.launch.py use_gui:=true

# 不启动RViz（用于headless模式）
ros2 launch mmk2_fake_moveit_config demo.launch.py use_rviz:=false
```

## 与真实机器人的关系

```
mmk2_fake_moveit_config (本包)     mmk2_moveit_config (真实机器人)
├── config/                        ├── config/
│   └── moveit_controllers.yaml    │   ├── moveit_controllers.yaml  ← 控制器名称一致！
│       (控制器名称一致)            │   ├── mmk2.srdf                ← 共享
├── scripts/                       │   ├── kinematics.yaml          ← 共享
│   └── fake_joint_driver.py       │   └── joint_limits.yaml        ← 共享
└── launch/                        └── launch/
    └── demo.launch.py                 └── demo.launch.py
```

## 开发工作流

1. 在虚拟机器人上开发和测试功能
2. 功能验证通过后，直接切换到真实机器人运行
3. 无需修改任何控制器相关代码

```bash
# 开发阶段 - 使用虚拟机器人
ros2 launch mmk2_fake_moveit_config demo.launch.py

# 部署阶段 - 使用真实机器人
ros2 launch mmk2_moveit_config demo.launch.py
```
