import scipy.spatial
import matplotlib.pyplot as plt
import pathlib
import numpy as np
import scipy.ndimage
import skimage.segmentation
import imageio.v3 as iio
import yaml

from geometry_msgs.msg import Vector3
from f1tenth_gym.envs.track import Raceline
from agent.state import nearest_point_on_trajectory
from geometry_msgs.msg import Pose

class MapInfo:
    def __init__(self, origin: Pose, resolution: float):
        self.origin: Pose = origin
        self.resolution: float = resolution

def load_map_info(map_folder_path: str, map_name: str) -> MapInfo:
    with open(f"{map_folder_path}/{map_name}_map.yaml", 'r') as file:
        config = yaml.safe_load(file)

        resolution = float(config['resolution'])

        origin = Pose()
        origin.position.x = float(config['origin'][0])
        origin.position.y = float(config['origin'][1])
        origin.position.z = float(config['origin'][2])

        origin.orientation.x = 0.0
        origin.orientation.y = 0.0
        origin.orientation.z = 0.0
        origin.orientation.w = 1.0
        return MapInfo(origin, resolution)

    return None


def load_map(map_folder_path: str, map_name: str) -> np.ndarray:
    map = 255 - np.flip(iio.imread(f"{map_folder_path}/{map_name}_map.png"), 0)
    return (map > 0).astype(np.uint8)


def load_centerline(map_folder_path: str, map_name: str) -> np.ndarray:
    centerline = Raceline.from_centerline_file(pathlib.Path(f"{map_folder_path}/{map_name}_centerline.csv"))
    return np.flip(np.stack([centerline.xs, centerline.ys], dtype=np.float64).T, 0)


def load_raceline(map_folder_path: str, map_name: str) -> np.ndarray:
    raceline = Raceline.from_raceline_file(pathlib.Path(f"{map_folder_path}/{map_name}_raceline.csv"))
    return np.flip(np.stack([raceline.xs, raceline.ys], dtype=np.float64).T, 0)


def map_to_grid_coordinates(map_info: MapInfo, coordinate: Vector3) -> np.ndarray:
    row = ((coordinate.y - map_info.origin.position.y) / map_info.resolution)
    column = ((coordinate.x - map_info.origin.position.x) / map_info.resolution)
    return np.array([row, column], dtype=int)


def grid_to_map_coordinates(map_info: MapInfo, coordinate: np.ndarray) -> np.ndarray:
    x = (coordinate[1] * map_info.resolution) + map_info.origin.position.x
    y = (coordinate[0] * map_info.resolution) + map_info.origin.position.y
    return np.array([x, y])


def dilate_map(map: np.ndarray, radius: int) -> np.ndarray:
    dilation_footprint = np.zeros((2 * radius, 2 * radius))
    rx, ry = np.indices(dilation_footprint.shape)
    radius_grid = (rx - radius)**2 + (ry - radius)**2
    dilation_footprint[radius_grid <= radius**2] = 1
    return scipy.ndimage.grey_dilation(map, footprint=dilation_footprint)


def calculate_curvature(p1, p2, p3):
    L1 = np.linalg.norm(p2 - p1)
    L2 = np.linalg.norm(p3 - p2)
    L3 = np.linalg.norm(p3 - p1)

    A = 0.5 * np.linalg.det(np.array([[p1[0], p1[1], 1],
                                       [p2[0], p2[1], 1],
                                       [p3[0], p3[1], 1]]))

    if L1 * L2 * L3 != 0:
        curvature = 2 * A / (L1 * L2 * L3)
    else:
        curvature = 0
    return curvature

if __name__ == "__main__":
    MAP_NAME: str = "Spielberg"
    MAP_FOLDER_PATH: str = pathlib.Path(__file__).parent.resolve() / "f1tenth_racetracks" / MAP_NAME
    ANGLE_DIFFERENCE_STEP: int = 100

    map_info = load_map_info(MAP_FOLDER_PATH, MAP_NAME)
    map = load_map(MAP_FOLDER_PATH, MAP_NAME)
    centerline = load_centerline(MAP_FOLDER_PATH, MAP_NAME)
    raceline = load_raceline(MAP_FOLDER_PATH, MAP_NAME)

    dilated_map = dilate_map(map, 6)

    map_origin_in_grid = map_to_grid_coordinates(map_info, Vector3())
    track_points = skimage.segmentation.flood(dilated_map, (map_origin_in_grid[0], map_origin_in_grid[1]))

    costmap = np.full_like(map, 100.0, dtype=float)
    centerline_index_map = np.full_like(map, -1, dtype=int)

    alpha_map = np.full_like(map, 0, dtype=float)
    alpha_map[track_points] = 1.0

    curvature_map_visual = np.full_like(map, 0.0, dtype=float)

    for p in np.argwhere(track_points):
        _, distance_from_raceline, _, centerline_index = nearest_point_on_trajectory(grid_to_map_coordinates(map_info, p), centerline)
        costmap[p[0], p[1]] = 100 * distance_from_raceline
        centerline_index_map[p[0], p[1]] = centerline_index

        centerline_point = centerline[centerline_index, :]
        next_centerline_point = centerline[(centerline_index + ANGLE_DIFFERENCE_STEP) % centerline.shape[0], :]
        next_next_centerline_point = centerline[(centerline_index + 2*ANGLE_DIFFERENCE_STEP) % centerline.shape[0], :]

        curvature_map_visual[p[0], p[1]] = calculate_curvature(centerline_point, next_centerline_point, next_next_centerline_point)


    curvature_map = np.zeros((centerline.shape[0]), dtype=float)
    for p in range(0, centerline.shape[0]):
        centerline_point = centerline[p, :]
        next_centerline_point = centerline[(p + ANGLE_DIFFERENCE_STEP) % centerline.shape[0], :]
        next_next_centerline_point = centerline[(p + 2*ANGLE_DIFFERENCE_STEP) % centerline.shape[0], :]

        curvature_map[p] = calculate_curvature(centerline_point, next_centerline_point, next_next_centerline_point)


    np.save(f'{MAP_NAME}_costmap', costmap)
    np.save(f'{MAP_NAME}_curvature_map', curvature_map)
    np.save(f'{MAP_NAME}_centerline_index_map', centerline_index_map)

    plt.imshow(curvature_map_visual, cmap='cool', interpolation='none', alpha=alpha_map)
    plt.colorbar()
    plt.show()
