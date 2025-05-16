import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from visualization_msgs.msg import Marker
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from tf2_ros import Buffer, TransformListener, TransformException
import numpy as np

class RelativeObstaclePublisher(Node):
    def __init__(self):
        super().__init__('relative_obstacle_publisher')
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.cloud_pub = self.create_publisher(PointCloud2,
                                               '/virtual_obstacles',
                                               qos)
        self.marker_pub = self.create_publisher(Marker,
                                                '/visualization_marker',
                                                qos)
        # TF buffer to get robot pose in map frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 장애물 상대 위치 (r, θ[rad]) — 필요에 따라 여기나 파라미터로 변경
        self.r = 7.0        # 2m 앞 
        self.theta = math.radians(60)  # 로봇 앞에서 30도 우측

        # 박스 크기, 해상도
        self.box_size = 1.0  # 0.5m 정사각형
        self.resolution = 0.1

        self.timer = self.create_timer(0.5, self.publish_obstacle)

    def publish_obstacle(self):
        try:
            # robot_base_link → map transform
            t = self.tf_buffer.lookup_transform(
                'map',        # target
                'base_link',  # source
                rclpy.time.Time())
        except TransformException as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return

        # 1) 로봇 기준 극좌표 → map 프레임 Cartesian
        # robot 위치 (tx, ty), robot yaw (from quaternion)
        q = t.transform.rotation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y),
                         1.0 - 2.0*(q.y*q.y + q.z*q.z))
        robot_x = t.transform.translation.x
        robot_y = t.transform.translation.y

        # 상대 위치
        obs_x = robot_x + self.r * math.cos(yaw + self.theta)
        obs_y = robot_y + self.r * math.sin(yaw + self.theta)

        # 2) PointCloud2 박스 생성
        half = self.box_size / 2.0
        xs = np.arange(obs_x - half, obs_x + half, self.resolution)
        ys = np.arange(obs_y - half, obs_y + half, self.resolution)
        pts = [[x, y, 0.0] for x in xs for y in ys]

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'
        cloud = create_cloud_xyz32(header, pts)
        self.cloud_pub.publish(cloud)
        # self.r = self.r - 0.1
        # self.theta += 1

        # 3) RViz Marker
        marker = Marker()
        marker.header = header
        marker.ns = 'rel_obs'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = obs_x
        marker.pose.position.y = obs_y
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.box_size
        marker.scale.y = self.box_size
        marker.scale.z = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.5
        marker.lifetime.sec = 0
        self.marker_pub.publish(marker)

        self.get_logger().info(
            f'Published obstacle at map ({obs_x:.2f}, {obs_y:.2f})')

def main(args=None):
    rclpy.init(args=args)
    node = RelativeObstaclePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()