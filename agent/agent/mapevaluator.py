import rclpy
import rclpy.node
import rclpy.publisher
import rclpy.subscription
import rclpy.timer
import rclpy.qos

from tf2_ros import TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Vector3

import numpy as np

class MapEvaluator(rclpy.node.Node):
    def __init__(self):
        super().__init__('map_evaluator')

        self.declare_parameter('ego_namespace', 'ego_racecar')
        self.declare_parameter('opponent_namespace', 'opp_racecar')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('reference_frame', 'map')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('costmap_topic', '/costmap')

        ego_namespace = self.get_parameter('ego_namespace').value
        opponent_namespace = self.get_parameter('opponent_namespace').value
        target_frame = self.get_parameter('target_frame').value
        map_topic = self.get_parameter('map_topic').value
        costmap_topic = self.get_parameter('costmap_topic').value

        self.ego_frame: str = f'{ego_namespace}/{target_frame}'
        self.opponent_frame: str = f'{opponent_namespace}/{target_frame}'
        self.reference_frame: str = self.get_parameter('reference_frame').value

        self.tf_buffer: Buffer = Buffer()
        self.tf_listener: TransformListener = TransformListener(self.tf_buffer, self)

        self.timer: rclpy.timer.Timer = self.create_timer(1, self.update)

        qos_profile = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                                           durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                                           history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                           depth=5)
        
        self.map_subscriber: rclpy.subscription.Subscription = self.create_subscription(OccupancyGrid, map_topic, self.update_map, qos_profile)
        self.map: OccupancyGrid = None

        self.costmap_publisher: rclpy.publisher.Publisher = self.create_publisher(OccupancyGrid, costmap_topic, qos_profile)
        self.costmap: np.ndarray = None

    def get_ego_position(self):
        try:
            t = self.tf_buffer.lookup_transform(self.ego_frame, self.reference_frame, rclpy.time.Time())
            return t.transform.translation
        except Exception as ex:
            self.get_logger().info(f'Could not transform {self.ego_frame} to {self.reference_frame}: {ex}')
            return None

    def get_opponent_position(self):
        try:
            t = self.tf_buffer.lookup_transform(self.opponent_frame, self.reference_frame, rclpy.time.Time())
            return t.transform.translation
        except Exception as ex:
            self.get_logger().info(f'Could not transform {self.opponent_frame} to {self.reference_frame}: {ex}')
            return None

    def update_map(self, map: OccupancyGrid):
        self.get_logger().error('Map updated')
        self.map = map

    def map_to_grid_coordinates(self, coordinate: Vector3) -> np.ndarray:
        row = (coordinate.y - self.map.info.origin.position.y) // self.map.info.resolution
        column = (coordinate.x - self.map.info.origin.position.x) // self.map.info.resolution
        return np.array([row, column], dtype=int)

    def update(self):
        if not self.map:
            return 
        ego_position = self.get_ego_position()
        opponent_position = self.get_opponent_position()
        if not ego_position or not opponent_position:
            return

        ego_grid_position = self.map_to_grid_coordinates(ego_position)
        opponent_grid_position = self.map_to_grid_coordinates(opponent_position)
        self.get_logger().info(f'Ego position {ego_position}, opponent position {opponent_position}')
        self.get_logger().info(f'Ego position grid {ego_grid_position}, opponent position grid {opponent_grid_position}')

        self.costmap = np.reshape(self.map.data, (self.map.info.height, self.map.info.width))

        self.costmap[opponent_grid_position[0] - 20 : opponent_grid_position[0] + 20, opponent_grid_position[1] - 20 : opponent_grid_position[1] + 20] = 100

        costmap = OccupancyGrid()
        costmap.data = self.costmap.reshape((-1)).tobytes()
        costmap.header = self.map.header
        costmap.info = self.map.info
        self.costmap_publisher.publish(costmap)

        


def main():
    rclpy.init()
    evaluator = MapEvaluator()
    rclpy.spin(evaluator)