# Demo URDF - 静态场景障碍物包

将URDF定义的静态场景作为MoveIt避障规划的障碍物。

## 工作原理

### 1. URDF结构

```
URDF文件
├── <link>           # 定义刚体
│   ├── <visual>     # 可视化几何体（用于显示）
│   └── <collision>  # 碰撞几何体（用于避障）★重要
└── <joint>          # 定义link之间的连接关系和位置
```

**关键点**：
- `<collision>` 元素定义了碰撞检测的几何体
- MoveIt只关心 `<collision>`，不关心 `<visual>`
- `<joint type="fixed">` 定义静态场景中各物体的位置

### 2. 坐标系关系

```
world (URDF根坐标系)
   │
   ├── table_top (桌面)
   │   ├── table_leg_fl
   │   ├── table_leg_fr
   │   └── ...
   │
   ├── back_wall (墙壁)
   │
   └── pillar (立柱)

base_link (机器人根坐标系)
   │
   └── ... (机器人各关节)
```

**需要通过静态TF将 `world` 和 `base_link` 对齐！**

### 3. 发布流程

```
┌─────────────────┐     ┌─────────────────────┐     ┌──────────────────┐
│   scene.urdf    │────▶│ publish_scene_to_   │────▶│  /planning_scene │
│   (URDF文件)    │     │ moveit.py (解析器)   │     │   (MoveIt话题)   │
└─────────────────┘     └─────────────────────┘     └──────────────────┘
                                │
                                ▼
                        ┌─────────────────┐
                        │ CollisionObject │
                        │ - id: 物体名称   │
                        │ - primitives:   │
                        │   Box/Cylinder  │
                        │ - poses: 位姿   │
                        └─────────────────┘
```

### 4. 支持的几何体

| URDF几何体 | MoveIt类型 | 参数 |
|------------|-----------|------|
| `<box size="x y z"/>` | BOX | [x, y, z] |
| `<cylinder radius="r" length="l"/>` | CYLINDER | [length, radius] |
| `<sphere radius="r"/>` | SPHERE | [radius] |
| `<mesh filename="..."/>` | MESH | (需要额外处理) |

## 文件结构

```
demo_urdf/
├── urdf/
│   └── scene.urdf              # 场景URDF定义
├── scripts/
│   └── publish_scene_to_moveit.py  # URDF转PlanningScene
├── launch/
│   ├── scene.launch.py         # 单独启动场景
│   └── moveit_with_scene.launch.py # MoveIt+场景
├── config/
├── meshes/                     # 自定义mesh文件（可选）
├── package.xml
├── CMakeLists.txt
└── README.md
```

## 使用方法

### 编译

```bash
cd /ros2_ws
colcon build --packages-select demo_urdf
source install/setup.bash
```

### 方式1：与MoveIt一起启动

```bash
ros2 launch demo_urdf moveit_with_scene.launch.py
```

### 方式2：单独启动场景（MoveIt已运行）

```bash
# 终端1：启动MoveIt
ros2 launch mmk2_moveit_config demo.launch.py

# 终端2：加载场景
ros2 launch demo_urdf scene.launch.py
```

### 方式3：使用自定义URDF

```bash
ros2 run demo_urdf publish_scene_to_moveit.py --ros-args \
    -p urdf_file:=/path/to/your/scene.urdf \
    -p frame_id:=base_link
```

## 自定义场景

### 添加新物体

编辑 `urdf/scene.urdf`：

```xml
<!-- 添加一个箱子 -->
<link name="my_box">
  <collision>
    <geometry>
      <box size="0.5 0.3 0.4"/>  <!-- 长x宽x高 -->
    </geometry>
  </collision>
</link>

<joint name="world_to_my_box" type="fixed">
  <parent link="world"/>
  <child link="my_box"/>
  <origin xyz="1.0 0.5 0.2" rpy="0 0 0"/>  <!-- 位置和旋转 -->
</joint>
```

### 添加圆柱障碍物

```xml
<link name="my_cylinder">
  <collision>
    <geometry>
      <cylinder radius="0.1" length="0.5"/>
    </geometry>
  </collision>
</link>

<joint name="world_to_my_cylinder" type="fixed">
  <parent link="world"/>
  <child link="my_cylinder"/>
  <origin xyz="0.8 -0.3 0.25" rpy="0 0 0"/>
</joint>
```

## RViz中查看

1. 添加 `RobotModel` 显示
   - Description Topic: `/demo_scene/robot_description`
   
2. 或在 `MotionPlanning` 插件中
   - Scene Objects 会显示所有碰撞物体

## 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `urdf_file` | - | URDF文件路径（必需） |
| `frame_id` | `world` | 场景参考坐标系 |
| `scene_name` | `urdf_scene` | 物体ID前缀 |
| `publish_rate` | `1.0` | 发布频率(Hz) |
| `one_shot` | `True` | 是否只发布一次 |

## 注意事项

1. **坐标系对齐**：确保URDF的根坐标系与机器人的世界坐标系对齐
2. **碰撞体大小**：适当增大碰撞体可提供安全余量
3. **更新场景**：修改URDF后需重新编译并重启节点
4. **性能**：大量小物体可能影响规划性能，建议合并简化
