import numpy as np

from geometry_msgs.msg import Pose2D
from interfaces.msg import Trajectory

def pose_to_array(pose: Pose2D) -> np.ndarray:
    return np.array([pose.x, pose.x, pose.theta])


def array_to_pose(array: np.ndarray) -> Pose2D:
    assert array.shape == (3,), "Invalid array size"
    return Pose2D(x=array[0], y=array[1], theta=array[2])


def trajectory_to_array(trajectory: Trajectory) -> np.ndarray:
    result = []
    for pose in trajectory.poses:
        result.append(pose_to_array(pose))
    return np.array(result)


def array_to_trajectory(array: np.ndarray) -> Trajectory:
    assert len(array.shape) == 2 and array.shape[1] == 3, "Invalid array size"
    result = Trajectory()
    for i in range(array.shape[0]):
        result.poses.append(array_to_pose(array[i, :]))
    return result

