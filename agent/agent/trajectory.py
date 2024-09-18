from geometry_msgs.msg import Pose2D

from dataclasses import dataclass

@dataclass
class Trajectory:
    poses: list[Pose2D]

@dataclass
class TrajectoryEvaluation:
    progress: float
    cost: float
    collision: bool

    def __init__(self, progress: float = 0.0, cost: float = 0.0, collision: bool = False):
        self.progress = progress
        self.cost = cost
        self.collision = collision