# MMK2 URDF Package

独立的 MMK2 机器人 URDF 可视化包，包含完整的机器人模型和所有 mesh 文件。

## 功能特点

- 包含完整的 MMK2 机器人 URDF（底盘 + 双臂 + 夹爪）
- 所有 mesh 文件已整合到本包内
- 可独立运行，无需依赖其他 description 包
- 支持在线 URDF 可视化工具

## 使用方法

### 在 RViz 中显示

```bash
# 编译
cd /home/ubuntu/ws/mmk2_ws
colcon build --packages-select mmk2_urdf
source install/setup.bash

# 启动可视化
ros2 launch mmk2_urdf display.launch.py
```

### 导出用于在线可视化

运行导出脚本生成可用于在线 viewer 的文件：

```bash
cd /home/ubuntu/ws/mmk2_ws/src/mmk2_urdf
./scripts/export_for_viewer.sh
```

这将在 `/tmp/mmk2_export/` 生成：
- `mmk2.urdf` - 使用相对路径的 URDF 文件
- `meshes/` - 所有 mesh 文件
- `mmk2_model.zip` - 打包文件，可直接上传到 viewer.robotsfan.com

## 包结构

```
mmk2_urdf/
├── urdf/
│   └── mmk2.urdf           # 完整的机器人 URDF
├── meshes/
│   ├── slamtec_athena/     # 底盘 mesh
│   ├── mmk2/               # 头部和升降柱 mesh
│   ├── airbot_play_v3_1/   # 机械臂 mesh
│   ├── G2/                 # 夹爪 mesh
│   └── flange/             # 法兰 mesh
├── launch/
│   └── display.launch.py   # RViz 显示 launch 文件
├── config/
│   └── display.rviz        # RViz 配置文件
└── scripts/
    └── export_for_viewer.sh # 导出脚本
```

## Mesh 文件来源

| 组件 | 原始包 | 目录 |
|------|--------|------|
| 底盘 | mmk2_description | meshes/slamtec_athena/ |
| 头部/升降柱 | mmk2_description | meshes/mmk2/ |
| 机械臂 | airbot_description | meshes/airbot_play_v3_1/ |
| 夹爪 G2 | airbot_description | meshes/G2/ |
| 法兰 | airbot_description | meshes/flange/ |
