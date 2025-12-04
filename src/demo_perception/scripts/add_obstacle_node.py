#!/usr/bin/env python3
"""
障碍物添加节点 - 在MoveIt2 Planning Scene中添加一个Box障碍物
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, PlanningScene
from std_msgs.msg import Header


class AddObstacleNode(Node):
    def __init__(self):
        super().__init__('add_obstacle_node')
        
        # 声明参数
        self.declare_parameter('box_x', 0.5)  # box位置 x
        self.declare_parameter('box_y', 0.0)  # box位置 y
        self.declare_parameter('box_z', 1.2)  # box位置 z
        self.declare_parameter('box_size_x', 0.1)  # box尺寸 x
        self.declare_parameter('box_size_y', 0.4)  # box尺寸 y
        self.declare_parameter('box_size_z', 1.6)  # box尺寸 z
        self.declare_parameter('frame_id', 'base_link')  # 参考坐标系
        self.declare_parameter('obstacle_id', 'obstacle_box')  # 障碍物ID
        
        # 创建Planning Scene发布器
        self.planning_scene_pub = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            10
        )
        
        # 等待MoveIt启动
        self.get_logger().info('等待MoveIt启动...')
        self.create_timer(3.0, self.add_obstacle_callback)
        
    def add_obstacle_callback(self):
        """添加障碍物到场景"""
        box_x = self.get_parameter('box_x').value
        box_y = self.get_parameter('box_y').value
        box_z = self.get_parameter('box_z').value
        box_size_x = self.get_parameter('box_size_x').value
        box_size_y = self.get_parameter('box_size_y').value
        box_size_z = self.get_parameter('box_size_z').value
        frame_id = self.get_parameter('frame_id').value
        obstacle_id = self.get_parameter('obstacle_id').value
        
        # 创建CollisionObject
        collision_object = CollisionObject()
        collision_object.header = Header()
        collision_object.header.frame_id = frame_id
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = obstacle_id
        
        # 定义Box形状
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [box_size_x, box_size_y, box_size_z]
        
        # 定义Box位置
        box_pose = Pose()
        box_pose.position = Point(x=box_x, y=box_y, z=box_z)
        box_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(box_pose)
        collision_object.operation = CollisionObject.ADD
        
        # 创建PlanningScene消息
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.world.collision_objects.append(collision_object)
        
        self.planning_scene_pub.publish(planning_scene)
        
        self.get_logger().info(
            f'已添加障碍物Box: 位置=({box_x}, {box_y}, {box_z}), '
            f'尺寸=({box_size_x}, {box_size_y}, {box_size_z})'
        )
        
        # 持续发布确保障碍物存在
        self.create_timer(1.0, self.republish_obstacle)
        
    def republish_obstacle(self):
        """定期重新发布障碍物"""
        box_x = self.get_parameter('box_x').value
        box_y = self.get_parameter('box_y').value
        box_z = self.get_parameter('box_z').value
        box_size_x = self.get_parameter('box_size_x').value
        box_size_y = self.get_parameter('box_size_y').value
        box_size_z = self.get_parameter('box_size_z').value
        frame_id = self.get_parameter('frame_id').value
        obstacle_id = self.get_parameter('obstacle_id').value
        
        collision_object = CollisionObject()
        collision_object.header.frame_id = frame_id
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = obstacle_id
        
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [box_size_x, box_size_y, box_size_z]
        
        box_pose = Pose()
        box_pose.position = Point(x=box_x, y=box_y, z=box_z)
        box_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(box_pose)
        collision_object.operation = CollisionObject.ADD
        
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.world.collision_objects.append(collision_object)
        
        self.planning_scene_pub.publish(planning_scene)


def main(args=None):
    rclpy.init(args=args)
    node = AddObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
