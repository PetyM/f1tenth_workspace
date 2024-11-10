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

from f1tenth_gym.envs.track import Raceline
from state import nearest_point_on_trajectory

import pathlib
import numpy as np
import scipy.ndimage
import skimage.segmentation

class MapEvaluator(rclpy.node.Node):
    def __init__(self):
        super().__init__('map_evaluator')

        self.declare_parameter('ego_namespace', 'ego_racecar')
        self.declare_parameter('opponent_namespace', 'opp_racecar')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('reference_frame', 'map')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('costmap_topic', '/costmap')
        self.declare_parameter('opponent_present', False)
        self.declare_parameter('map_name', '')
        self.declare_parameter('map_folder_path', '')

        self.opponent_present: bool = self.get_parameter('opponent_present').value

        ego_namespace = self.get_parameter('ego_namespace').value
        target_frame = self.get_parameter('target_frame').value
        map_topic = self.get_parameter('map_topic').value
        costmap_topic = self.get_parameter('costmap_topic').value

        self.ego_frame: str = f'{ego_namespace}/{target_frame}'
        self.reference_frame: str = self.get_parameter('reference_frame').value

        if self.opponent_present:
            opponent_namespace = self.get_parameter('opponent_namespace').value
            self.opponent_frame: str = f'{opponent_namespace}/{target_frame}'

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


        map_name = self.get_parameter('map_name').value
        map_folder_path = self.get_parameter('map_folder_path').value

        raceline_file = pathlib.Path(f"{map_folder_path}/{map_name}_raceline.csv")
        raceline = Raceline.from_raceline_file(raceline_file)
        self.raceline: np.ndarray = np.stack([raceline.xs, raceline.ys], dtype=np.float64).T

        ego_mask: np.ndarray = np.zeros((self.evaluation_area_size, self.evaluation_area_size))
        ego_mask[self.mask_offset - 4 : self.mask_offset + 4, self.mask_offset - 4 : self.mask_offset + 4] = 1
        ego_mask = scipy.ndimage.gaussian_filter(ego_mask, sigma=20)
        self.ego_mask: np.ndarray = -100 * (ego_mask / ego_mask.max())

        if self.opponent_present:
            opponent_mask = np.zeros((self.evaluation_area_size, self.evaluation_area_size))
            opponent_mask[self.mask_offset - 4 : self.mask_offset + 4, self.mask_offset - 4 : self.mask_offset + 4] = 1
            opponent_mask = scipy.ndimage.gaussian_filter(opponent_mask, sigma=20)
            self.opponent_mask: np.ndarray = 100 * (opponent_mask / opponent_mask.max())


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
        costmap[costmap >= 0.45] = 1
        costmap[costmap < 0.45] = 0

        skimage.segmentation.flood_fill(costmap, (0, 0), 255, in_place=True)
        skimage.segmentation.flood_fill(costmap, (800, 800), 255, in_place=True)

        track_points = np.argwhere(costmap == 0)
        for p in track_points:
                _, distance_from_centerline, _, _ = nearest_point_on_trajectory(self.grid_to_map_coordinates(p), self.raceline)
                costmap[p] = distance_from_centerline
        costmap[track_points] = 255 * (costmap[track_points] / np.max(costmap[track_points]))

        self.map = costmap
        self.get_logger().error("MAP UPDATE DONE")


    def map_to_grid_coordinates(self, coordinate: Vector3) -> np.ndarray:
        row = ((coordinate.y - self.map_info.origin.position.y) / self.map_info.resolution)
        column = ((coordinate.x - self.map_info.origin.position.x) / self.map_info.resolution)
        return np.array([row, column], dtype=int)

    def grid_to_map_coordinates(self, coordinate: np.ndarray) -> np.ndarray:
        x = (coordinate[1] * self.map_info.resolution) + self.map_info.origin.position.x
        y = (coordinate[0] * self.map_info.resolution) + self.map_info.origin.position.y
        return np.array([x, y])

    def update(self):
        if self.map is None:
            return  
        start = self.get_clock().now()
        ego_position = self.get_ego_position()
        if not ego_position:
            return
        
        ego_grid_position = self.map_to_grid_coordinates(ego_position)

        if self.opponent_present:
            opponent_position = self.get_opponent_position()
            if not opponent_position:
                return
            
            opponent_grid_position = self.map_to_grid_coordinates(opponent_position)

        costmap = np.zeros_like(self.map)
        costmap[ego_grid_position[0] - self.mask_offset - 1 : ego_grid_position[0] + self.mask_offset, ego_grid_position[1] - self.mask_offset - 1 : ego_grid_position[1] + self.mask_offset] = self.ego_mask

        if self.opponent_present:
            opponent_costmap = np.zeros_like(self.map)
            opponent_costmap[opponent_grid_position[0] - self.mask_offset - 1: opponent_grid_position[0] + self.mask_offset, opponent_grid_position[1] - self.mask_offset -1 : opponent_grid_position[1] + self.mask_offset] = self.opponent_mask

            costmap = np.add(costmap, opponent_costmap)


        costmap = np.add(costmap, self.map)


        self.costmap = costmap

        costmap = OccupancyGrid()
        costmap.data = self.costmap.reshape((-1)).tobytes()
        costmap.header.frame_id = "map"
        costmap.header.stamp = self.get_clock().now().to_msg()
        costmap.info = self.map_info
        self.costmap_publisher.publish(costmap)

        # self.get_logger().info(f"(update) took {(self.get_clock().now() - start).nanoseconds / 1e6} ms")


def main():
    rclpy.init()
    evaluator = MapEvaluator()
    rclpy.spin(evaluator)