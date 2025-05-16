import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import numpy as np

class MapWithObsMarker(Node):
    def __init__(self):
        super().__init__('map_with_obs_marker')
        # QoS: Latched처럼 동작하도록 DURABILITY=TRANSIENT_LOCAL 설정
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # 원본 맵 구독
        self.sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_cb, 10)
        # 수정된 맵 Latched 퍼블리시
        self.pub_map = self.create_publisher(
            OccupancyGrid, '/map_with_obs', qos)
        # Marker 퍼블리시 (기본 QoS로 충분)
        self.pub_marker = self.create_publisher(
            Marker, '/visualization_marker', 10)

        self.obs_min_x = 3.0
        self.obs_max_x = 5.0
        self.obs_min_y = -2.0
        self.obs_max_y = 0.0

    def map_cb(self, msg: OccupancyGrid):
        w, h = msg.info.width, msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((h, w))

        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y
        res = msg.info.resolution

        ix0 = max(0, int((self.obs_min_x - ox) / res))
        ix1 = min(w-1, int((self.obs_max_x - ox) / res))
        iy0 = max(0, int((self.obs_min_y - oy) / res))
        iy1 = min(h-1, int((self.obs_max_y - oy) / res))

        data[iy0:iy1+1, ix0:ix1+1] = 100

        new_map = OccupancyGrid()
        new_map.header = msg.header
        new_map.info = msg.info
        new_map.data = list(data.flatten())
        self.pub_map.publish(new_map)

        marker = Marker()
        marker.header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id='map'
        )
        marker.ns = 'static_obs'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = (self.obs_min_x + self.obs_max_x) / 2.0
        marker.pose.position.y = (self.obs_min_y + self.obs_max_y) / 2.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.obs_max_x - self.obs_min_x
        marker.scale.y = self.obs_max_y - self.obs_min_y
        marker.scale.z = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.5
        self.pub_marker.publish(marker)

        self.get_logger().info(
            f'Published latched /map_with_obs and Marker at center'
        )

def main():
    rclpy.init()
    node = MapWithObsMarker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()