import scipy.spatial
import rclpy
import rclpy.node
import rclpy.publisher
import rclpy.subscription
import rclpy.time
import rclpy.timer
import rclpy.qos

from tf2_ros import TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Transform

from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Vector3

import numpy as np
import scipy.ndimage

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

        self.tf_buffer: Buffer = Buffer(rclpy.time.Duration(seconds=1), self)
        self.tf_listener: TransformListener = TransformListener(self.tf_buffer, self)

        self.timer: rclpy.timer.Timer = self.create_timer(0.01, self.update)

        qos_profile = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                                           durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                                           history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                           depth=5)
        
        self.map_subscriber: rclpy.subscription.Subscription = self.create_subscription(OccupancyGrid, map_topic, self.update_map, qos_profile)
        self.map: np.ndarray = None
        self.map_info: MapMetaData = None

        self.costmap_publisher: rclpy.publisher.Publisher = self.create_publisher(OccupancyGrid, costmap_topic, qos_profile)
        self.costmap: np.ndarray = None
        self.car_size: float = 0.4

        self.evaluation_area_size: int = 199
        self.mask_offset: int = int((self.evaluation_area_size - 1) // 2)


        opponent_mask = np.zeros((self.evaluation_area_size, self.evaluation_area_size))
        opponent_mask[self.mask_offset - 4 : self.mask_offset + 4, self.mask_offset - 4 : self.mask_offset + 4] = 1
        opponent_mask = scipy.ndimage.gaussian_filter(opponent_mask, sigma=20)
        self.opponent_mask: np.ndarray = 100 * (opponent_mask / opponent_mask.max())

        ego_mask: np.ndarray = np.zeros((self.evaluation_area_size, self.evaluation_area_size))
        ego_mask[self.mask_offset - 4 : self.mask_offset + 4, self.mask_offset - 4 : self.mask_offset + 4] = 1
        ego_mask = scipy.ndimage.gaussian_filter(ego_mask, sigma=20)
        self.ego_mask: np.ndarray = -100 * (ego_mask / ego_mask.max())

    def get_ego_position(self):
        try:
            t = self.tf_buffer.lookup_transform(self.reference_frame, self.ego_frame, rclpy.time.Time())
            return t.transform.translation
        except Exception as ex:
            self.get_logger().info(f'Could not transform {self.ego_frame} to {self.reference_frame}: {ex}')
            return None


    def get_opponent_position(self):
        try:
            t = self.tf_buffer.lookup_transform(self.reference_frame, self.opponent_frame, rclpy.time.Time())
            return t.transform.translation
        except Exception as ex:
            self.get_logger().info(f'Could not transform {self.opponent_frame} to {self.reference_frame}: {ex}')
            return None


    def update_map(self, map: OccupancyGrid):
        self.get_logger().error('Map updated')
        self.map_info = map.info
        self.get_logger().warn(f"{map.info.origin.position.x}, {map.info.origin.position.y}, {map.info.origin.position.z}")
        self.get_logger().warn(f"{map.info.origin.orientation.x}, {map.info.origin.orientation.y}, {map.info.origin.orientation.z}, {map.info.origin.orientation.w}")

        costmap = np.reshape(map.data, (map.info.height, map.info.width))

        dilation_diameter = int(np.ceil((1.2 * self.car_size) / self.map_info.resolution))
        if dilation_diameter % 2 == 0:
            dilation_diameter += 1
        dilation_footprint_radius = np.ceil(dilation_diameter / 2)

        dilation_footprint = np.zeros((dilation_diameter, dilation_diameter))
        rx, ry = np.indices(dilation_footprint.shape)
        radius_grid = (rx - dilation_footprint_radius)**2 + (ry - dilation_footprint_radius)**2
        dilation_footprint[radius_grid <= dilation_footprint_radius**2] = 1

        self.map = scipy.ndimage.morphology.grey_dilation(costmap, footprint=dilation_footprint)



    def map_to_grid_coordinates(self, coordinate: Vector3) -> np.ndarray:
        row = ((coordinate.y - self.map_info.origin.position.y) / self.map_info.resolution)
        column = ((coordinate.x - self.map_info.origin.position.x) / self.map_info.resolution)
        return np.array([row, column], dtype=int)


    def update(self):
        if self.map is None:
            return  
        start = self.get_clock().now()
        ego_position = self.get_ego_position()
        opponent_position = self.get_opponent_position()
        if not ego_position or not opponent_position:
            return

        ego_grid_position = self.map_to_grid_coordinates(ego_position)
        opponent_grid_position = self.map_to_grid_coordinates(opponent_position)

        ego_costmap = np.zeros_like(self.map)
        ego_costmap[ego_grid_position[0] - self.mask_offset - 1 : ego_grid_position[0] + self.mask_offset, ego_grid_position[1] - self.mask_offset - 1 : ego_grid_position[1] + self.mask_offset] = self.ego_mask

        opponent_costmap = np.zeros_like(self.map)
        opponent_costmap[opponent_grid_position[0] - self.mask_offset - 1: opponent_grid_position[0] + self.mask_offset, opponent_grid_position[1] - self.mask_offset -1 : opponent_grid_position[1] + self.mask_offset] = self.opponent_mask

        costmap = np.add(ego_costmap, opponent_costmap)
        costmap[self.map > 0] = 100

        self.costmap = costmap

        costmap = OccupancyGrid()
        costmap.data = self.costmap.reshape((-1)).tobytes()
        costmap.header.frame_id = "map"
        costmap.header.stamp = self.get_clock().now().to_msg()
        costmap.info = self.map_info
        self.costmap_publisher.publish(costmap)

        self.get_logger().info(f"(update) took {(self.get_clock().now() - start).nanoseconds / 1e6} ms")


def main():
    rclpy.init()
    evaluator = MapEvaluator()
    rclpy.spin(evaluator)