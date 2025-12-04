#!/usr/bin/env python3
"""
工位场景发布到MoveIt节点

功能：
1. 解析URDF文件中的collision元素（支持mesh）
2. 将每个collision转换为MoveIt的CollisionObject
3. 发布到/planning_scene话题

基于demo_urdf的实现，扩展支持mesh文件
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
import xml.etree.ElementTree as ET
import os
import numpy as np
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import Pose, Point, Quaternion
from shape_msgs.msg import SolidPrimitive, Mesh, MeshTriangle
from moveit_msgs.msg import CollisionObject, PlanningScene
from std_msgs.msg import Header
from ament_index_python.packages import get_package_share_directory


class URDFScenePublisher(Node):
    """将URDF场景发布到MoveIt PlanningScene（支持mesh）"""
    
    def __init__(self):
        super().__init__('urdf_scene_publisher')
        
        # 声明参数
        self.declare_parameter('urdf_file', '')
        self.declare_parameter('frame_id', 'base_link')  # 场景的参考坐标系
        self.declare_parameter('scene_name', 'ws_scene')  # 场景名称前缀
        self.declare_parameter('publish_rate', 1.0)  # 发布频率(Hz)
        self.declare_parameter('mesh_scale', 0.001)  # mesh缩放（STL通常是mm）
        
        # 获取参数
        self.urdf_file = self.get_parameter('urdf_file').value
        self.frame_id = self.get_parameter('frame_id').value
        self.scene_name = self.get_parameter('scene_name').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.mesh_scale = self.get_parameter('mesh_scale').value
        
        if not self.urdf_file:
            self.get_logger().error('必须指定urdf_file参数！')
            return
        
        if not os.path.exists(self.urdf_file):
            self.get_logger().error(f'URDF文件不存在: {self.urdf_file}')
            return
        
        # 发布PlanningScene
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.planning_scene_pub = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            qos
        )
        
        # 解析URDF
        self.links = {}
        self.joints = {}
        self.parse_urdf()
        
        # 计算每个link在世界坐标系中的位姿
        self.link_poses = {}
        self.calculate_link_poses()
        
        # 预加载mesh
        self.cached_meshes = {}
        self.preload_meshes()
        
        # 持续发布场景
        self.create_timer(1.0 / self.publish_rate, self.publish_scene)
        
        self.get_logger().info(f'工位场景发布节点已启动')
        self.get_logger().info(f'  URDF文件: {self.urdf_file}')
        self.get_logger().info(f'  参考坐标系: {self.frame_id}')
        self.get_logger().info(f'  找到 {len(self.links)} 个links')
    
    def parse_urdf(self):
        """解析URDF文件"""
        try:
            tree = ET.parse(self.urdf_file)
            root = tree.getroot()
            
            # 解析所有link
            for link in root.findall('link'):
                link_name = link.get('name')
                collision = link.find('collision')
                
                link_info = {
                    'name': link_name,
                    'collision': None
                }
                
                if collision is not None:
                    geometry = collision.find('geometry')
                    origin = collision.find('origin')
                    
                    collision_offset = self.parse_origin(origin)
                    geom_info = self.parse_geometry(geometry)
                    
                    if geom_info:
                        link_info['collision'] = {
                            'geometry': geom_info,
                            'offset': collision_offset
                        }
                
                self.links[link_name] = link_info
            
            # 解析所有joint
            for joint in root.findall('joint'):
                joint_name = joint.get('name')
                joint_type = joint.get('type')
                parent = joint.find('parent').get('link')
                child = joint.find('child').get('link')
                origin = joint.find('origin')
                
                self.joints[joint_name] = {
                    'name': joint_name,
                    'type': joint_type,
                    'parent': parent,
                    'child': child,
                    'origin': self.parse_origin(origin)
                }
            
            self.get_logger().info(f'解析完成: {len(self.links)} links, {len(self.joints)} joints')
            
        except Exception as e:
            self.get_logger().error(f'解析URDF失败: {e}')
    
    def parse_origin(self, origin_elem):
        """解析origin元素"""
        if origin_elem is None:
            return (np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0]))
        
        xyz_str = origin_elem.get('xyz', '0 0 0')
        xyz = np.array([float(x) for x in xyz_str.split()])
        
        rpy_str = origin_elem.get('rpy', '0 0 0')
        rpy = np.array([float(x) for x in rpy_str.split()])
        
        rot = Rotation.from_euler('xyz', rpy)
        quat = rot.as_quat()
        
        return (xyz, quat)
    
    def parse_geometry(self, geometry_elem):
        """解析geometry元素"""
        if geometry_elem is None:
            return None
        
        # Box
        box = geometry_elem.find('box')
        if box is not None:
            size_str = box.get('size', '1 1 1')
            size = [float(x) for x in size_str.split()]
            return {'type': 'box', 'size': size}
        
        # Cylinder
        cylinder = geometry_elem.find('cylinder')
        if cylinder is not None:
            radius = float(cylinder.get('radius', '0.5'))
            length = float(cylinder.get('length', '1.0'))
            return {'type': 'cylinder', 'radius': radius, 'length': length}
        
        # Sphere
        sphere = geometry_elem.find('sphere')
        if sphere is not None:
            radius = float(sphere.get('radius', '0.5'))
            return {'type': 'sphere', 'radius': radius}
        
        # Mesh
        mesh = geometry_elem.find('mesh')
        if mesh is not None:
            filename = mesh.get('filename', '')
            scale_str = mesh.get('scale', '1 1 1')
            scale = [float(x) for x in scale_str.split()]
            return {'type': 'mesh', 'filename': filename, 'scale': scale}
        
        return None
    
    def resolve_mesh_path(self, filename):
        """解析mesh文件路径（支持package://格式）"""
        if filename.startswith('package://'):
            # 解析 package://pkg_name/path
            path = filename[len('package://'):]
            pkg_name = path.split('/')[0]
            rel_path = '/'.join(path.split('/')[1:])
            try:
                pkg_share = get_package_share_directory(pkg_name)
                return os.path.join(pkg_share, rel_path)
            except Exception as e:
                self.get_logger().error(f'无法找到包 {pkg_name}: {e}')
                return None
        elif filename.startswith('file://'):
            return filename[len('file://'):]
        else:
            return filename
    
    def load_stl_mesh(self, filepath):
        """加载STL文件并转换为Mesh消息"""
        mesh = Mesh()
        
        try:
            with open(filepath, 'rb') as f:
                # 跳过80字节头部
                f.read(80)
                num_triangles = np.frombuffer(f.read(4), dtype=np.uint32)[0]
                
                self.get_logger().info(f'加载STL: {num_triangles} 个三角形')
                
                vertices = []
                vertex_map = {}
                
                for _ in range(num_triangles):
                    f.read(12)  # 跳过法向量
                    
                    triangle = MeshTriangle()
                    triangle_indices = []
                    
                    for _ in range(3):
                        vertex_data = np.frombuffer(f.read(12), dtype=np.float32)
                        vertex = tuple(float(v * self.mesh_scale) for v in vertex_data)
                        
                        if vertex not in vertex_map:
                            vertex_map[vertex] = len(vertices)
                            point = Point()
                            point.x, point.y, point.z = vertex
                            vertices.append(point)
                        
                        triangle_indices.append(vertex_map[vertex])
                    
                    triangle.vertex_indices = triangle_indices
                    mesh.triangles.append(triangle)
                    f.read(2)  # 跳过属性
                
                mesh.vertices = vertices
                self.get_logger().info(f'网格加载完成: {len(vertices)} 个顶点')
                
        except Exception as e:
            self.get_logger().error(f'加载STL失败: {e}')
            return None
        
        return mesh
    
    def preload_meshes(self):
        """预加载所有mesh文件"""
        for link_name, link_info in self.links.items():
            if link_info['collision'] is None:
                continue
            
            geom = link_info['collision']['geometry']
            if geom['type'] == 'mesh':
                filename = geom['filename']
                if filename not in self.cached_meshes:
                    filepath = self.resolve_mesh_path(filename)
                    if filepath and os.path.exists(filepath):
                        mesh = self.load_stl_mesh(filepath)
                        if mesh:
                            self.cached_meshes[filename] = mesh
                            self.get_logger().info(f'缓存mesh: {filename}')
    
    def calculate_link_poses(self):
        """计算每个link在世界坐标系中的位姿"""
        child_links = set(j['child'] for j in self.joints.values())
        root_links = set(self.links.keys()) - child_links
        
        for root in root_links:
            self.link_poses[root] = (np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0]))
            self._propagate_poses(root)
        
        self.get_logger().info(f'计算了 {len(self.link_poses)} 个link的世界位姿')
    
    def _propagate_poses(self, parent_link):
        """递归计算子link的位姿"""
        parent_pos, parent_quat = self.link_poses[parent_link]
        parent_rot = Rotation.from_quat(parent_quat)
        
        for joint in self.joints.values():
            if joint['parent'] == parent_link:
                child_link = joint['child']
                joint_pos, joint_quat = joint['origin']
                joint_rot = Rotation.from_quat(joint_quat)
                
                child_pos = parent_pos + parent_rot.apply(joint_pos)
                child_rot = parent_rot * joint_rot
                child_quat = child_rot.as_quat()
                
                self.link_poses[child_link] = (child_pos, child_quat)
                self._propagate_poses(child_link)
    
    def create_collision_object(self, link_name, link_info, obj_id):
        """为一个link创建CollisionObject"""
        if link_info['collision'] is None:
            return None
        
        collision = link_info['collision']
        geom = collision['geometry']
        offset_pos, offset_quat = collision['offset']
        
        if link_name not in self.link_poses:
            return None
        
        link_pos, link_quat = self.link_poses[link_name]
        link_rot = Rotation.from_quat(link_quat)
        offset_rot = Rotation.from_quat(offset_quat)
        
        # 计算collision在世界坐标系中的位姿
        world_pos = link_pos + link_rot.apply(offset_pos)
        world_rot = link_rot * offset_rot
        world_quat = world_rot.as_quat()
        
        # 创建CollisionObject
        co = CollisionObject()
        co.header.frame_id = self.frame_id
        co.header.stamp = self.get_clock().now().to_msg()
        co.id = obj_id
        co.operation = CollisionObject.ADD
        
        # 设置位姿
        pose = Pose()
        pose.position = Point(x=float(world_pos[0]), y=float(world_pos[1]), z=float(world_pos[2]))
        pose.orientation = Quaternion(
            x=float(world_quat[0]),
            y=float(world_quat[1]),
            z=float(world_quat[2]),
            w=float(world_quat[3])
        )
        
        # 根据几何类型创建
        if geom['type'] == 'mesh':
            filename = geom['filename']
            if filename in self.cached_meshes:
                co.meshes.append(self.cached_meshes[filename])
                co.mesh_poses.append(pose)
            else:
                return None
        else:
            primitive = SolidPrimitive()
            if geom['type'] == 'box':
                primitive.type = SolidPrimitive.BOX
                primitive.dimensions = geom['size']
            elif geom['type'] == 'cylinder':
                primitive.type = SolidPrimitive.CYLINDER
                primitive.dimensions = [geom['length'], geom['radius']]
            elif geom['type'] == 'sphere':
                primitive.type = SolidPrimitive.SPHERE
                primitive.dimensions = [geom['radius']]
            else:
                return None
            
            co.primitives.append(primitive)
            co.primitive_poses.append(pose)
        
        return co
    
    def publish_scene(self):
        """发布场景到PlanningScene"""
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        
        obj_count = 0
        for link_name, link_info in self.links.items():
            if link_info['collision'] is not None:
                obj_id = f'{self.scene_name}_{link_name}'
                co = self.create_collision_object(link_name, link_info, obj_id)
                if co is not None:
                    planning_scene.world.collision_objects.append(co)
                    obj_count += 1
        
        if obj_count > 0:
            self.planning_scene_pub.publish(planning_scene)


def main(args=None):
    rclpy.init(args=args)
    node = URDFScenePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
