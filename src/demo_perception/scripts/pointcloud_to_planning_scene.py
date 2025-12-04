#!/usr/bin/env python3
"""
点云转PlanningScene节点
将点云中的物体检测出来并添加到MoveIt的PlanningScene中作为障碍物
支持简单的地面过滤和聚类
支持tf2坐标变换
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, TransformStamped
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, PlanningScene
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import struct

# tf2
from tf2_ros import Buffer, TransformListener, TransformException


class PointCloudToPlanningScene(Node):
    def __init__(self):
        super().__init__('pointcloud_to_planning_scene')
        
        # 声明参数
        self.declare_parameter('pointcloud_topic', '/head_camera/pointcloud')
        self.declare_parameter('planning_scene_topic', '/planning_scene')
        self.declare_parameter('voxel_size', 0.05)  # 体素大小(米)
        self.declare_parameter('min_points_per_voxel', 3)  # 每个体素最小点数
        self.declare_parameter('ground_height', 0.05)  # 地面高度阈值
        self.declare_parameter('max_height', 2.0)  # 最大高度
        self.declare_parameter('min_height', 0.0)  # 最小高度
        self.declare_parameter('frame_id', 'base_link')  # 障碍物坐标系
        self.declare_parameter('update_rate', 2.0)  # 更新频率(Hz)
        self.declare_parameter('obstacle_padding', 0.005)  # 障碍物膨胀（减小以紧贴点云）
        self.declare_parameter('use_boxes', False)  # 使用单个体素而非聚类Box（更紧贴）
        self.declare_parameter('cluster_tolerance', 0.05)  # 聚类容差
        self.declare_parameter('max_distance', 1.2)  # 最大前方距离
        
        # 获取参数
        self.pointcloud_topic = self.get_parameter('pointcloud_topic').value
        self.planning_scene_topic = self.get_parameter('planning_scene_topic').value
        self.voxel_size = self.get_parameter('voxel_size').value
        self.min_points_per_voxel = self.get_parameter('min_points_per_voxel').value
        self.ground_height = self.get_parameter('ground_height').value
        self.max_height = self.get_parameter('max_height').value
        self.min_height = self.get_parameter('min_height').value
        self.frame_id = self.get_parameter('frame_id').value
        self.update_rate = self.get_parameter('update_rate').value
        self.obstacle_padding = self.get_parameter('obstacle_padding').value
        self.use_boxes = self.get_parameter('use_boxes').value
        self.cluster_tolerance = self.get_parameter('cluster_tolerance').value
        self.max_distance = self.get_parameter('max_distance').value
        
        # 存储最新的点云
        self.latest_pointcloud = None
        self.obstacle_id_counter = 0
        self.current_obstacles = set()
        
        # tf2 Buffer和Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # QoS配置
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 订阅点云
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            self.pointcloud_topic,
            self.pointcloud_callback,
            qos
        )
        
        # 发布PlanningScene
        self.planning_scene_pub = self.create_publisher(
            PlanningScene,
            self.planning_scene_topic,
            10
        )
        
        # 发布可视化标记
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/obstacle_markers',
            10
        )
        
        # 定时更新
        self.update_timer = self.create_timer(
            1.0 / self.update_rate,
            self.update_planning_scene
        )
        
        self.get_logger().info(f'点云转PlanningScene节点已启动')
        self.get_logger().info(f'  点云话题: {self.pointcloud_topic}')
        self.get_logger().info(f'  体素大小: {self.voxel_size}m')
        self.get_logger().info(f'  更新频率: {self.update_rate}Hz')
    
    def pointcloud_callback(self, msg):
        """接收点云"""
        self.latest_pointcloud = msg
    
    def parse_pointcloud(self, msg):
        """解析PointCloud2消息为numpy数组"""
        # 获取点云字段信息
        fields = {f.name: f for f in msg.fields}
        
        # 检查必要字段
        if 'x' not in fields or 'y' not in fields or 'z' not in fields:
            return None
        
        # 解析点云数据
        point_step = msg.point_step
        num_points = msg.width * msg.height
        
        # 提取xyz坐标
        data = np.frombuffer(msg.data, dtype=np.uint8)
        
        x_offset = fields['x'].offset
        y_offset = fields['y'].offset
        z_offset = fields['z'].offset
        
        points = np.zeros((num_points, 3), dtype=np.float32)
        
        for i in range(num_points):
            base = i * point_step
            points[i, 0] = np.frombuffer(data[base + x_offset:base + x_offset + 4], dtype=np.float32)[0]
            points[i, 1] = np.frombuffer(data[base + y_offset:base + y_offset + 4], dtype=np.float32)[0]
            points[i, 2] = np.frombuffer(data[base + z_offset:base + z_offset + 4], dtype=np.float32)[0]
        
        # 过滤无效点
        valid_mask = np.isfinite(points).all(axis=1)
        points = points[valid_mask]
        
        return points
    
    def voxelize_points(self, points):
        """将点云体素化"""
        if len(points) == 0:
            return []
        
        # 计算体素索引
        voxel_indices = np.floor(points / self.voxel_size).astype(np.int32)
        
        # 统计每个体素的点数
        unique_voxels, counts = np.unique(voxel_indices, axis=0, return_counts=True)
        
        # 过滤点数太少的体素
        valid_mask = counts >= self.min_points_per_voxel
        valid_voxels = unique_voxels[valid_mask]
        
        # 计算体素中心
        voxel_centers = (valid_voxels.astype(np.float32) + 0.5) * self.voxel_size
        
        return voxel_centers
    
    def filter_points(self, points):
        """过滤点云（去除地面、超出范围的点等）"""
        if len(points) == 0:
            return points
        
        # 高度过滤
        height_mask = (points[:, 2] > self.min_height + self.ground_height) & \
                      (points[:, 2] < self.max_height)
        
        # 前方距离过滤（X轴正方向为前方）
        # 只保留前方max_distance米内的点
        forward_mask = (points[:, 0] > 0) & (points[:, 0] < self.max_distance)
        
        # 横向距离过滤（Y轴，左右各0.5米）
        lateral_mask = np.abs(points[:, 1]) < 0.5
        
        combined_mask = height_mask & forward_mask & lateral_mask
        return points[combined_mask]
    
    def cluster_voxels(self, voxel_centers):
        """简单聚类，将相邻体素合并为更大的Box"""
        if len(voxel_centers) == 0:
            return []
        
        # 简单方法：计算包围盒
        # 对于更复杂的场景，可以使用DBSCAN等聚类算法
        
        if not self.use_boxes:
            # 返回单个体素
            boxes = []
            for center in voxel_centers:
                boxes.append({
                    'center': center,
                    'size': np.array([self.voxel_size, self.voxel_size, self.voxel_size])
                })
            return boxes
        
        # 使用简单的网格聚类
        boxes = []
        visited = set()
        
        for i, center in enumerate(voxel_centers):
            if i in visited:
                continue
            
            # 找到相邻的体素
            cluster_indices = [i]
            visited.add(i)
            
            for j in range(i + 1, len(voxel_centers)):
                if j in visited:
                    continue
                
                # 检查是否相邻
                dist = np.linalg.norm(voxel_centers[j] - center)
                if dist < self.cluster_tolerance:
                    cluster_indices.append(j)
                    visited.add(j)
            
            # 计算聚类的包围盒
            cluster_points = voxel_centers[cluster_indices]
            min_pt = np.min(cluster_points, axis=0) - self.voxel_size / 2
            max_pt = np.max(cluster_points, axis=0) + self.voxel_size / 2
            
            center = (min_pt + max_pt) / 2
            size = max_pt - min_pt + self.obstacle_padding * 2
            
            boxes.append({
                'center': center,
                'size': size
            })
        
        return boxes
    
    def transform_points(self, points, source_frame, target_frame):
        """将点从source_frame变换到target_frame"""
        try:
            # 获取变换
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # 提取平移和旋转
            t = transform.transform.translation
            q = transform.transform.rotation
            
            # 四元数转旋转矩阵
            # q = [x, y, z, w]
            qx, qy, qz, qw = q.x, q.y, q.z, q.w
            
            # 旋转矩阵
            R = np.array([
                [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
                [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
                [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
            ])
            
            # 平移向量
            T = np.array([t.x, t.y, t.z])
            
            # 变换点云: p' = R * p + T
            transformed_points = (R @ points.T).T + T
            
            return transformed_points
            
        except TransformException as e:
            self.get_logger().warn(f'坐标变换失败 ({source_frame} -> {target_frame}): {e}', throttle_duration_sec=2.0)
            return None

    def update_planning_scene(self):
        """更新PlanningScene"""
        if self.latest_pointcloud is None:
            return
        
        try:
            # 解析点云
            points = self.parse_pointcloud(self.latest_pointcloud)
            if points is None or len(points) == 0:
                return
            
            # 获取点云的源坐标系
            source_frame = self.latest_pointcloud.header.frame_id
            
            # 如果源坐标系和目标坐标系不同，进行变换
            if source_frame != self.frame_id:
                points = self.transform_points(points, source_frame, self.frame_id)
                if points is None:
                    return
            
            # 过滤点云
            filtered_points = self.filter_points(points)
            if len(filtered_points) == 0:
                return
            
            # 体素化
            voxel_centers = self.voxelize_points(filtered_points)
            if len(voxel_centers) == 0:
                return
            
            # 聚类
            boxes = self.cluster_voxels(voxel_centers)
            
            # 创建PlanningScene消息
            planning_scene = PlanningScene()
            planning_scene.is_diff = True
            
            # 首先删除旧的障碍物
            for old_id in self.current_obstacles:
                remove_obj = CollisionObject()
                remove_obj.id = old_id
                remove_obj.operation = CollisionObject.REMOVE
                planning_scene.world.collision_objects.append(remove_obj)
            
            self.current_obstacles.clear()
            
            # 添加新的障碍物
            marker_array = MarkerArray()
            
            for i, box in enumerate(boxes):
                # 创建CollisionObject
                obj_id = f'pointcloud_obstacle_{self.obstacle_id_counter}_{i}'
                self.current_obstacles.add(obj_id)
                
                collision_object = CollisionObject()
                collision_object.header.frame_id = self.frame_id
                collision_object.header.stamp = self.get_clock().now().to_msg()
                collision_object.id = obj_id
                
                # 定义Box形状
                primitive = SolidPrimitive()
                primitive.type = SolidPrimitive.BOX
                primitive.dimensions = [float(box['size'][0]), float(box['size'][1]), float(box['size'][2])]
                
                # 定义位置
                pose = Pose()
                pose.position = Point(x=float(box['center'][0]), y=float(box['center'][1]), z=float(box['center'][2]))
                pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                
                collision_object.primitives.append(primitive)
                collision_object.primitive_poses.append(pose)
                collision_object.operation = CollisionObject.ADD
                
                planning_scene.world.collision_objects.append(collision_object)
                
                # 创建可视化标记
                marker = Marker()
                marker.header.frame_id = self.frame_id
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = 'obstacles'
                marker.id = i
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose = pose
                marker.scale = Vector3(x=float(box['size'][0]), y=float(box['size'][1]), z=float(box['size'][2]))
                marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.5)
                marker.lifetime.sec = 1
                
                marker_array.markers.append(marker)
            
            # 发布
            self.planning_scene_pub.publish(planning_scene)
            self.marker_pub.publish(marker_array)
            
            self.obstacle_id_counter += 1
            
            self.get_logger().debug(f'已更新 {len(boxes)} 个障碍物')
            
        except Exception as e:
            self.get_logger().error(f'更新PlanningScene时出错: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToPlanningScene()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
