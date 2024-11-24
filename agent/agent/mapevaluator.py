import PIL.Image
import PIL.ImageShow
import scipy.spatial
import rclpy
import rclpy.node
import rclpy.publisher
import rclpy.service
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

from geometry_msgs.msg import Pose2D

from custom_interfaces.msg import Trajectory, TrajectoryEvaluation
from custom_interfaces.srv import EvaluateTrajectories

import pathlib
import numpy as np
import scipy.ndimage
import skimage.segmentation
from matplotlib import pyplot as plt
import imageio.v3 as iio
import yaml

class MapEvaluator(rclpy.node.Node):
    def __init__(self):
        super().__init__('map_evaluator')

        self.declare_parameter('ego_namespace', 'ego_racecar')
        self.declare_parameter('opponent_namespace', 'opp_racecar')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('reference_frame', 'map')
        self.declare_parameter('costmap_topic', '/costmap')
        self.declare_parameter('opponent_present', False)
        self.declare_parameter('map_name', '')
        self.declare_parameter('map_folder_path', '')

        self.opponent_present: bool = self.get_parameter('opponent_present').value

        ego_namespace = self.get_parameter('ego_namespace').value
        target_frame = self.get_parameter('target_frame').value
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
        
        self.map: np.ndarray = None
        self.map_info: MapMetaData = MapMetaData()

        self.costmap_publisher: rclpy.publisher.Publisher = self.create_publisher(OccupancyGrid, costmap_topic, qos_profile)
        self.costmap: np.ndarray = None
        self.car_size: float = 0.4

        self.evaluation_area_size: int = 199
        self.mask_offset: int = int((self.evaluation_area_size - 1) // 2)

        map_name = self.get_parameter('map_name').value
        map_folder_path = self.get_parameter('map_folder_path').value

        self.prepare_costmap(map_folder_path, map_name)

        if self.opponent_present:
            opponent_mask = np.zeros((self.evaluation_area_size, self.evaluation_area_size))
            opponent_mask[self.mask_offset - 4 : self.mask_offset + 4, self.mask_offset - 4 : self.mask_offset + 4] = 1
            opponent_mask = scipy.ndimage.gaussian_filter(opponent_mask, sigma=20)
            self.opponent_mask: np.ndarray = 100 * (opponent_mask / opponent_mask.max())
            

        self.evaluate_service: rclpy.service.Service = self.create_service(EvaluateTrajectories, 'evaluate_trajectories', self.evaluate_trajectories)


    def prepare_costmap(self, map_folder_path: str, map_name: str):
        with open(f"{map_folder_path}/{map_name}_map.yaml", 'r') as file:
            config = yaml.safe_load(file)

            self.map_info.resolution = float(config['resolution'])

            self.map_info.origin.position.x = float(config['origin'][0])
            self.map_info.origin.position.y = float(config['origin'][1])
            self.map_info.origin.position.z = float(config['origin'][2])

            self.map_info.origin.orientation.x = 0.0
            self.map_info.origin.orientation.y = 0.0
            self.map_info.origin.orientation.z = 0.0
            self.map_info.origin.orientation.w = 1.0
            self.map_info.map_load_time = self.get_clock().now().to_msg()

        costmap_path = pathlib.Path(f'{map_name}_costmap.png')
        if costmap_path.exists():
            self.get_logger().error(f'Costmap found. Using previously generated costmap ({costmap_path})')

            self.map = iio.imread(costmap_path)
            
        else:
            self.get_logger().error('Costmap NOT found. Calculating costmap...')

            raceline_file = pathlib.Path(f"{map_folder_path}/{map_name}_raceline.csv")
            raceline = Raceline.from_raceline_file(raceline_file)
            raceline_points = np.stack([raceline.xs, raceline.ys], dtype=np.float64).T

            costmap = 255 - np.flip(iio.imread(f"{map_folder_path}/{map_name}_map.png"), 0)
            costmap = (costmap > 0).astype(np.uint8)

            # assume map origin (0,0) is position on track (cars start at (0,0) by default)
            map_origin_in_grid = self.map_to_grid_coordinates(Vector3())

            track_points = skimage.segmentation.flood(costmap, (map_origin_in_grid[0], map_origin_in_grid[1]))

            final_costmap = np.full_like(costmap, 100)
            for p in np.argwhere(track_points):
                _, distance_from_raceline, _, _ = nearest_point_on_trajectory(self.grid_to_map_coordinates(p), raceline_points)
                final_costmap[p[0], p[1]] = 100 * min(distance_from_raceline, 2.55)
            final_costmap[track_points] = 100 * (final_costmap[track_points] / final_costmap[track_points].max())

            self.map = final_costmap
            iio.imwrite(costmap_path, self.map)

        self.map_info.width = self.map.shape[0]
        self.map_info.height = self.map.shape[1]
            

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


    def map_to_grid_coordinates(self, coordinate: Vector3|Pose2D) -> np.ndarray:
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

        self.get_logger().info(f"(update) took {(self.get_clock().now() - start).nanoseconds / 1e6} ms")


    def evaluate_trajectories(self, request: EvaluateTrajectories.Request, response: EvaluateTrajectories.Response) -> EvaluateTrajectories.Response:
        self.get_logger().info(f'Evaluating {len(request.trajectories)} trajectories')
        costmap: np.ndarray = self.costmap.copy()

        values: list[TrajectoryEvaluation] = []

        trajectory: Trajectory
        for trajectory in request.trajectories:
            self.get_logger().info(f'Evaluating trajectory of {len(trajectory.poses)} poses')

            evaluation = TrajectoryEvaluation()
            
            pose: Pose2D
            for pose in trajectory.poses:
                pose_in_grid = self.map_to_grid_coordinates(pose)
                
                pose_value = costmap[pose_in_grid[0], pose_in_grid[1]]

                if pose_value >= 100: # collision
                    evaluation.collision = True
 
                evaluation.trajectory_cost = evaluation.trajectory_cost + pose_value

            evaluation.progress = np.sqrt((trajectory.poses[-1].x - trajectory.poses[0].x)**2 + (trajectory.poses[-1].y - trajectory.poses[0].y)**2 )

            self.get_logger().info(f'{evaluation.progress=}, {evaluation.trajectory_cost=}, {evaluation.collision=}')
            values.append(evaluation)
            
        response.values = values
        self.get_logger().info(f'Evaluating done.')
        return response


def main():
    rclpy.init()
    evaluator = MapEvaluator()
    rclpy.spin(evaluator)