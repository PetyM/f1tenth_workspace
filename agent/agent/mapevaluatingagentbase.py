import scipy.spatial
import rclpy
import rclpy.node
import rclpy.publisher
import rclpy.service
import rclpy.subscription
import rclpy.time
import rclpy.timer
import rclpy.qos

from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Vector3

from f1tenth_gym.envs.track import Raceline
from state import nearest_point_on_trajectory

from geometry_msgs.msg import Pose2D


import pathlib
import numpy as np
import scipy.ndimage
import skimage.segmentation
import imageio.v3 as iio
import yaml

from agent.agentbase import AgentBase
from agent.trajectory import Trajectory, TrajectoryEvaluation

class MapEvaluatingAgentBase(AgentBase):
    def __init__(self):
        super().__init__()

        self.declare_parameter('costmap_topic', 'costmap')
        costmap_topic = self.get_parameter('costmap_topic').value

        self.declare_parameter('map_name', '')
        map_name = self.get_parameter('map_name').value

        self.declare_parameter('map_folder_path', '')
        map_folder_path = self.get_parameter('map_folder_path').value

        self.declare_parameter('map_log', False)
        self.map_log: bool = self.get_parameter('map_log').value

        self.timer_costmap_update: rclpy.timer.Timer = self.create_timer(0.01, self.update_costmap)

        qos_profile = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                                           durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                                           history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                           depth=5)
        
        self.costmap_base: np.ndarray = None
        self.centerline_index_map: np.ndarray = None
        self.curvature_map: np.ndarray = None
        self.map_info: MapMetaData = MapMetaData()

        self.costmap_publisher: rclpy.publisher.Publisher = self.create_publisher(OccupancyGrid, f'{self.agent_namespace}/{costmap_topic}', qos_profile)
        self.costmap: np.ndarray = None
        self.car_size: float = 0.4

        self.evaluation_area_size: int = 199
        self.mask_offset: int = int((self.evaluation_area_size - 1) // 2)

        self.prepare_costmap(map_folder_path, map_name)

        if self.opponent_present:
            opponent_mask = np.zeros((self.evaluation_area_size, self.evaluation_area_size))
            opponent_mask[self.mask_offset - 4 : self.mask_offset + 4, self.mask_offset - 4 : self.mask_offset + 4] = 1
            opponent_mask = scipy.ndimage.gaussian_filter(opponent_mask, sigma=20)
            self.opponent_mask: np.ndarray = 100 * (opponent_mask / opponent_mask.max())


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

        costmap_path = pathlib.Path(f'{map_name}_costmap.npy')
        if costmap_path.exists():
            self.costmap_base = np.load(costmap_path)
        else:
            self.get_logger().fatal('MapEvaluatingAgentBase.prepare_costmap: Costmap NOT found.')

        centerline_index_map_path = pathlib.Path(f'{map_name}_centerline_index_map.npy')
        if centerline_index_map_path.exists():
            self.centerline_index_map = np.load(centerline_index_map_path)
        else:
            self.get_logger().fatal('MapEvaluatingAgentBase.prepare_costmap: Centerline index map NOT found.')

        curvature_map_path = pathlib.Path(f'{map_name}_curvature_map.npy')
        if curvature_map_path.exists():
            self.curvature_map = np.load(curvature_map_path)
        else:
            self.get_logger().fatal('MapEvaluatingAgentBase.prepare_costmap: Curvature map NOT found.')


        self.map_info.width = self.costmap_base.shape[0]
        self.map_info.height = self.costmap_base.shape[1]
        self.costmap = self.costmap_base


    def map_to_grid_coordinates(self, coordinate: Vector3|Pose2D) -> np.ndarray:
        row = ((coordinate.y - self.map_info.origin.position.y) / self.map_info.resolution)
        column = ((coordinate.x - self.map_info.origin.position.x) / self.map_info.resolution)
        return np.array([row, column], dtype=int)


    def grid_to_map_coordinates(self, coordinate: np.ndarray) -> np.ndarray:
        x = (coordinate[1] * self.map_info.resolution) + self.map_info.origin.position.x
        y = (coordinate[0] * self.map_info.resolution) + self.map_info.origin.position.y
        return np.array([x, y])


    def update_costmap(self):
        if self.costmap_base is None:
            return  
        start = self.get_clock().now()

        opponent_grid_position = self.map_to_grid_coordinates(Pose2D(x=self.opp_state[0], y=self.opp_state[1], theta=self.opp_state[4]))

        costmap = np.zeros_like(self.costmap_base)

        if self.opponent_present:
            opponent_costmap = np.zeros_like(self.costmap_base)
            opponent_costmap[opponent_grid_position[0] - self.mask_offset - 1: opponent_grid_position[0] + self.mask_offset, opponent_grid_position[1] - self.mask_offset -1 : opponent_grid_position[1] + self.mask_offset] = self.opponent_mask

            costmap = np.add(costmap, opponent_costmap)


        costmap = np.add(costmap, self.costmap_base)

        self.costmap = costmap

        costmap = OccupancyGrid()
        costmap.data = self.costmap.reshape((-1)).tobytes()
        costmap.header.frame_id = "map"
        costmap.header.stamp = self.get_clock().now().to_msg()
        costmap.info = self.map_info

        self.costmap_publisher.publish(costmap)
        self.map_log and self.get_logger().info(f"MapEvaluatingAgentBase.update_costmap: Took {(self.get_clock().now() - start).nanoseconds / 1e6} ms")


    def evaluate_trajectories(self, trajectories: list[Trajectory]) -> list[TrajectoryEvaluation]:
        start = self.get_clock().now()
        self.map_log and self.get_logger().info(f'MapEvaluatingAgentBase.evaluate_trajectories: Evaluating {len(trajectories)} trajectories')
        costmap: np.ndarray = self.costmap.copy()

        values: list[TrajectoryEvaluation] = []
    
        initial_pose_in_grid: np.ndarray = self.map_to_grid_coordinates(trajectories[0].poses[0]).clip([0, 0], [self.map_info.width - 1, self.map_info.height - 1])

        initial_centerline_index = self.centerline_index_map[initial_pose_in_grid[0], initial_pose_in_grid[1]]

        trajectory: Trajectory
        for trajectory in trajectories:
            evaluation = TrajectoryEvaluation()
            
            pose: Pose2D
            for pose in trajectory.poses:
                pose_in_grid = self.map_to_grid_coordinates(pose).clip([0, 0], [self.map_info.width - 1, self.map_info.height - 1])
                
                pose_value = costmap[pose_in_grid[0], pose_in_grid[1]]

                if pose_value >= 100: # collision
                    evaluation.collision = True
 
                evaluation.cost = evaluation.cost + pose_value

            last_trajectory_pose_in_grid: np.ndarray = self.map_to_grid_coordinates(trajectory.poses[-1]).clip([0, 0], [self.map_info.width - 1, self.map_info.height - 1])
            centerline_index  = self.centerline_index_map[last_trajectory_pose_in_grid[0], last_trajectory_pose_in_grid[1]]

            if centerline_index < initial_centerline_index:
                evaluation.progress = float(centerline_index + initial_centerline_index - self.centerline.shape[0])
            else:
                evaluation.progress = float(centerline_index - initial_centerline_index)

            self.map_log and self.get_logger().info(f'{evaluation.progress=}, {evaluation.cost=}, {evaluation.collision=}')
            values.append(evaluation)
            
        self.map_log and self.get_logger().info(f"MapEvaluatingAgentBase.evaluate_trajectories: Evaluationg took {(self.get_clock().now() - start).nanoseconds / 1e6} ms")
        return values


    def get_curvature_for_position(self, position: np.ndarray) -> float:
        position_in_grid = self.map_to_grid_coordinates(Pose2D(x=position[0], y=position[1]))
        return self.curvature_map[position_in_grid[0], position_in_grid[1]]