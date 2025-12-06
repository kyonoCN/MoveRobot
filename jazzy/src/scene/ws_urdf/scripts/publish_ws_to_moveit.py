#!/usr/bin/env python3
"""
发布工位场景到MoveIt作为障碍物
将gongwei URDF中的mesh作为碰撞对象添加到planning scene
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, Point
from shape_msgs.msg import Mesh, MeshTriangle
from moveit_msgs.msg import CollisionObject, PlanningScene
from std_msgs.msg import Header

import numpy as np
from ament_index_python.packages import get_package_share_directory
import os


class WorkstationPublisher(Node):
    """发布工位作为MoveIt障碍物"""
    
    def __init__(self):
        super().__init__('workstation_publisher')
        
        # 参数 - 位置
        self.declare_parameter('position_x', 0.3)
        self.declare_parameter('position_y', 0.0)
        self.declare_parameter('position_z', 0.82)
        # 四元数：绕Z轴旋转180度 = (0, 0, 1, 0)
        self.declare_parameter('orientation_x', 0.0)
        self.declare_parameter('orientation_y', 0.0)
        self.declare_parameter('orientation_z', 1.0)
        self.declare_parameter('orientation_w', 0.0)
        self.declare_parameter('frame_id', 'base_link')  # 使用机器人的base_link坐标系
        self.declare_parameter('mesh_scale', 1.0)  # STL已经是米单位，无需缩放
        
        # 获取参数值
        self.position = [
            self.get_parameter('position_x').value,
            self.get_parameter('position_y').value,
            self.get_parameter('position_z').value
        ]
        self.orientation = [
            self.get_parameter('orientation_x').value,
            self.get_parameter('orientation_y').value,
            self.get_parameter('orientation_z').value,
            self.get_parameter('orientation_w').value
        ]
        self.frame_id = self.get_parameter('frame_id').value
        self.mesh_scale = self.get_parameter('mesh_scale').value
        
        # 发布器 (使用与add_obstacle_node.py相同的QoS)
        self.planning_scene_pub = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            10
        )
        
        # 缓存加载的mesh
        self.cached_mesh = None
        self.mesh_loaded = False
        
        # 定时器：首次加载mesh
        self.timer = self.create_timer(1.0, self.load_and_publish)
        
        self.get_logger().info('工位障碍物发布节点已启动')
        
    def load_stl_mesh(self, filepath):
        """加载STL文件并转换为Mesh消息"""
        mesh = Mesh()
        
        try:
            with open(filepath, 'rb') as f:
                # 跳过80字节头部
                f.read(80)
                # 读取三角形数量
                num_triangles = np.frombuffer(f.read(4), dtype=np.uint32)[0]
                
                self.get_logger().info(f'加载STL: {num_triangles} 个三角形')
                
                vertices = []
                vertex_map = {}
                
                for _ in range(num_triangles):
                    # 读取法向量（跳过）
                    f.read(12)
                    
                    triangle = MeshTriangle()
                    triangle_indices = []
                    
                    for _ in range(3):
                        # 读取顶点坐标
                        vertex_data = np.frombuffer(f.read(12), dtype=np.float32)
                        # 应用缩放
                        vertex = tuple(float(v * self.mesh_scale) for v in vertex_data)
                        
                        # 去重顶点
                        if vertex not in vertex_map:
                            vertex_map[vertex] = len(vertices)
                            point = Point()
                            point.x, point.y, point.z = vertex
                            vertices.append(point)
                        
                        triangle_indices.append(vertex_map[vertex])
                    
                    triangle.vertex_indices = triangle_indices
                    mesh.triangles.append(triangle)
                    
                    # 跳过属性字节
                    f.read(2)
                
                mesh.vertices = vertices
                self.get_logger().info(f'网格加载完成: {len(vertices)} 个顶点')
                
        except Exception as e:
            self.get_logger().error(f'加载STL失败: {e}')
            return None
            
        return mesh
    
    def load_and_publish(self):
        """首次加载mesh并开始持续发布"""
        if not self.mesh_loaded:
            # 获取mesh文件路径
            try:
                pkg_share = get_package_share_directory('ws_urdf')
                mesh_path = os.path.join(pkg_share, 'meshes', 'base_link.STL')
            except Exception as e:
                self.get_logger().error(f'无法找到ws_urdf包: {e}')
                return
            
            if not os.path.exists(mesh_path):
                self.get_logger().warn(f'Mesh文件不存在: {mesh_path}')
                return
            
            # 加载mesh
            self.cached_mesh = self.load_stl_mesh(mesh_path)
            if self.cached_mesh is None:
                return
            
            self.mesh_loaded = True
            self.get_logger().info(f'工位障碍物已加载，开始持续发布 (位置: {self.position}, 坐标系: {self.frame_id})')
            
            # 停止当前定时器，启动持续发布定时器
            self.timer.cancel()
            self.republish_timer = self.create_timer(1.0, self.publish_workstation)
        
        # 首次发布
        self.publish_workstation()
    
    def publish_workstation(self):
        """发布工位障碍物到planning scene"""
        if not self.mesh_loaded or self.cached_mesh is None:
            return
        
        # 创建碰撞对象
        collision_object = CollisionObject()
        collision_object.header = Header()
        collision_object.header.frame_id = self.frame_id
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = 'gongwei_workstation'
        collision_object.operation = CollisionObject.ADD
        
        # 设置mesh
        collision_object.meshes.append(self.cached_mesh)
        
        # 设置位姿
        pose = Pose()
        pose.position.x = self.position[0]
        pose.position.y = self.position[1]
        pose.position.z = self.position[2]
        pose.orientation.x = self.orientation[0]
        pose.orientation.y = self.orientation[1]
        pose.orientation.z = self.orientation[2]
        pose.orientation.w = self.orientation[3]
        collision_object.mesh_poses.append(pose)
        
        # 创建planning scene消息
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.world.collision_objects.append(collision_object)
        
        # 发布
        self.planning_scene_pub.publish(planning_scene)


def main(args=None):
    rclpy.init(args=args)
    node = WorkstationPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
