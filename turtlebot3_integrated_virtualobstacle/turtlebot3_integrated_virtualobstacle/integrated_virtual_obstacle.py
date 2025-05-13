#!/usr/bin/env python3
# ROS2 Python 노드: Nav2 코스트맵 반영용 PointCloud2와 RViz 시각화용 Marker를 통합 퍼블리시
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from visualization_msgs.msg import Marker
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import numpy as np


class IntegratedObstaclePublisher(Node):
    def __init__(self):
        super().__init__('integrated_obstacle_publisher')
        # 퍼블리셔 생성
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.cloud_pub_map = self.create_publisher(PointCloud2, 'virtual_obstacles', qos)
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', qos)
        self.timer = self.create_timer(0.2, self.publish_callback)
        # 박스 범위 및 해상도 설정
        self.x_min, self.x_max = 0.0, 1.0
        self.y_min, self.y_max = -2.0, -1.0
        self.resolution = 0.1

    def publish_callback(self):
        # 1) PointCloud2 생성 (Nav2 costmap 반영용)
        points = []
        xs = np.arange(self.x_min, self.x_max, self.resolution)
        ys = np.arange(self.y_min, self.y_max, self.resolution)
        for x in xs:
            for y in ys:
                points.append([x, y, 0.0])
        header_map = Header()
        header_map.stamp = self.get_clock().now().to_msg()
        header_map.frame_id = 'map'
        cloud_map = create_cloud_xyz32(header_map, points)
        self.cloud_pub_map.publish(cloud_map)

        # 2) Marker 생성 (RViz 시각화용)
        marker = Marker()
        marker.header = header_map
        marker.ns = 'virtual_box'  
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        # 박스 중심 및 크기 설정
        marker.pose.position.x = (self.x_min + self.x_max) / 2.0
        marker.pose.position.y = (self.y_min + self.y_max) / 2.0
        marker.pose.position.z = 1.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.x_max - self.x_min
        marker.scale.y = self.y_max - self.y_min
        marker.scale.z = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.5
        marker.lifetime.sec = 0
        self.marker_pub.publish(marker)
        # self.x_min -= 0.1
        # self.x_max -= 0.1
        self.get_logger().info(f'Published cloud ({len(points)} pts) and marker at box center')

def main(args=None):
    rclpy.init(args=args)
    node = IntegratedObstaclePublisher()
    # rclpy.spin(node)
    # rclpy.shutdown()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
