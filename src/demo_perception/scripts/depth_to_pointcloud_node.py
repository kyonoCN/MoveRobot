#!/usr/bin/env python3
"""
深度图转点云节点
订阅深度图和RGB图像，生成彩色点云并发布
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from message_filters import Subscriber, ApproximateTimeSynchronizer
import numpy as np
import struct


class DepthToPointCloudNode(Node):
    def __init__(self):
        super().__init__('depth_to_pointcloud_node')
        
        # 声明参数
        self.declare_parameter('depth_topic', '/head_camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('color_topic', '/head_camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/head_camera/color/camera_info')
        self.declare_parameter('pointcloud_topic', '/head_camera/pointcloud')
        self.declare_parameter('frame_id', 'head_camera_link')
        self.declare_parameter('depth_scale', 0.001)  # 深度值缩放因子，将mm转为m
        self.declare_parameter('max_depth', 5.0)  # 最大深度(米)
        self.declare_parameter('min_depth', 0.1)  # 最小深度(米)
        self.declare_parameter('voxel_size', 0.0)  # 体素滤波大小，0表示不滤波
        self.declare_parameter('downsample_factor', 1)  # 下采样因子
        
        # 获取参数
        self.depth_topic = self.get_parameter('depth_topic').value
        self.color_topic = self.get_parameter('color_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.pointcloud_topic = self.get_parameter('pointcloud_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.depth_scale = self.get_parameter('depth_scale').value
        self.max_depth = self.get_parameter('max_depth').value
        self.min_depth = self.get_parameter('min_depth').value
        self.voxel_size = self.get_parameter('voxel_size').value
        self.downsample_factor = self.get_parameter('downsample_factor').value
        
        # 相机内参
        self.camera_info = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        
        # QoS配置
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 订阅相机内参
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos
        )
        
        # 使用message_filters同步深度图和RGB图
        self.depth_sub = Subscriber(self, Image, self.depth_topic, qos_profile=qos)
        self.color_sub = Subscriber(self, Image, self.color_topic, qos_profile=qos)
        
        # 时间同步器
        self.sync = ApproximateTimeSynchronizer(
            [self.depth_sub, self.color_sub],
            queue_size=10,
            slop=0.1  # 100ms时间容差
        )
        self.sync.registerCallback(self.sync_callback)
        
        # 点云发布器
        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            self.pointcloud_topic,
            10
        )
        
        self.get_logger().info(f'深度图转点云节点已启动')
        self.get_logger().info(f'  深度话题: {self.depth_topic}')
        self.get_logger().info(f'  颜色话题: {self.color_topic}')
        self.get_logger().info(f'  点云话题: {self.pointcloud_topic}')
        self.get_logger().info(f'  坐标系: {self.frame_id}')
        
    def camera_info_callback(self, msg):
        """接收相机内参"""
        if self.camera_info is None:
            self.camera_info = msg
            # 提取内参矩阵 K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.get_logger().info(f'已获取相机内参: fx={self.fx:.2f}, fy={self.fy:.2f}, cx={self.cx:.2f}, cy={self.cy:.2f}')
    
    def sync_callback(self, depth_msg, color_msg):
        """同步回调：处理深度图和颜色图"""
        if self.camera_info is None:
            self.get_logger().warn('等待相机内参...', throttle_duration_sec=2.0)
            return
        
        try:
            # 解析深度图
            depth_image = self.parse_image(depth_msg)
            if depth_image is None:
                return
            
            # 解析颜色图
            color_image = self.parse_image(color_msg)
            if color_image is None:
                return
            
            # 生成点云
            pointcloud_msg = self.create_pointcloud(depth_image, color_image, depth_msg.header.stamp)
            
            if pointcloud_msg is not None:
                self.pointcloud_pub.publish(pointcloud_msg)
                
        except Exception as e:
            self.get_logger().error(f'处理图像时出错: {e}')
    
    def parse_image(self, msg):
        """解析ROS Image消息为numpy数组"""
        encoding = msg.encoding
        
        if encoding in ['16UC1', 'mono16']:
            # 深度图 (16位无符号整数)
            dtype = np.uint16
            channels = 1
        elif encoding in ['32FC1']:
            # 深度图 (32位浮点)
            dtype = np.float32
            channels = 1
        elif encoding in ['rgb8', 'bgr8']:
            dtype = np.uint8
            channels = 3
        elif encoding in ['rgba8', 'bgra8']:
            dtype = np.uint8
            channels = 4
        else:
            self.get_logger().warn(f'不支持的图像编码: {encoding}', throttle_duration_sec=5.0)
            return None
        
        # 转换为numpy数组
        image = np.frombuffer(msg.data, dtype=dtype)
        image = image.reshape((msg.height, msg.width, channels) if channels > 1 else (msg.height, msg.width))
        
        return image
    
    def create_pointcloud(self, depth_image, color_image, stamp):
        """从深度图和颜色图创建点云"""
        height, width = depth_image.shape[:2]
        
        # 下采样
        step = self.downsample_factor
        
        # 创建像素坐标网格
        u = np.arange(0, width, step)
        v = np.arange(0, height, step)
        u, v = np.meshgrid(u, v)
        
        # 获取深度值
        depth = depth_image[::step, ::step].astype(np.float32)
        
        # 处理深度值
        if depth_image.dtype == np.uint16:
            depth = depth * self.depth_scale  # 转换为米
        
        # 创建有效深度掩码
        valid_mask = (depth > self.min_depth) & (depth < self.max_depth) & (depth > 0)
        
        # 计算3D坐标 (相机坐标系)
        # X = (u - cx) * Z / fx
        # Y = (v - cy) * Z / fy
        # Z = depth
        z = depth[valid_mask]
        x = (u[valid_mask] - self.cx) * z / self.fx
        y = (v[valid_mask] - self.cy) * z / self.fy
        
        # 获取颜色值
        color_sampled = color_image[::step, ::step]
        
        if len(color_image.shape) == 3:
            if color_image.shape[2] == 4:  # RGBA/BGRA
                r = color_sampled[valid_mask, 0]
                g = color_sampled[valid_mask, 1]
                b = color_sampled[valid_mask, 2]
            else:  # RGB/BGR
                r = color_sampled[valid_mask, 0]
                g = color_sampled[valid_mask, 1]
                b = color_sampled[valid_mask, 2]
            
            # 如果是BGR格式，交换R和B
            if 'bgr' in color_image.dtype.name or True:  # 假设是BGR
                r, b = b, r
        else:
            # 灰度图
            r = g = b = color_sampled[valid_mask]
        
        # 创建点云数据
        num_points = len(x)
        if num_points == 0:
            return None
        
        # 打包RGB为单个uint32
        rgb = np.zeros(num_points, dtype=np.uint32)
        rgb = np.left_shift(r.astype(np.uint32), 16) | \
              np.left_shift(g.astype(np.uint32), 8) | \
              b.astype(np.uint32)
        
        # 创建结构化数组
        points = np.zeros(num_points, dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('rgb', np.uint32)
        ])
        points['x'] = x
        points['y'] = y
        points['z'] = z
        points['rgb'] = rgb
        
        # 创建PointCloud2消息
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.height = 1
        msg.width = num_points
        msg.is_dense = True
        msg.is_bigendian = False
        
        # 定义点云字段
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]
        msg.point_step = 16  # 4 + 4 + 4 + 4 bytes
        msg.row_step = msg.point_step * num_points
        msg.data = points.tobytes()
        
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = DepthToPointCloudNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
