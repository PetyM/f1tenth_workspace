import rclpy
import rclpy.client
import rclpy.publisher

import numpy as np
from f1tenth_gym.envs.track import Raceline
from model import vehicle_dynamics, integrate_state

from lineevaluator import LineEvaluation
from state import State

from geometry_msgs.msg import Pose2D
from interfaces.msg import Trajectory

import conversions

from agent.agent import Agent

from geometry_msgs.msg import Pose2D
from interfaces.srv import EvaluateTrajectories
from interfaces.msg import Trajectory
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs

class SamplingPlanner(Agent):
    def __init__(self) -> None:
        super().__init__()
        self.parameters: dict = {"mu": 1.0489, 
                                 "C_Sf": 4.718,
                                 "C_Sr": 5.4562,
                                 "lf": 0.15875,
                                 "lr": 0.17145,
                                 "h": 0.074,
                                 "m": 3.74,
                                 "I": 0.04712,
                                 "s_min": -0.4189,
                                 "s_max": 0.4189,
                                 "sv_min": -3.2,
                                 "sv_max": 3.2,
                                 "v_switch": 7.319,
                                 "a_max": 9.51,
                                 "v_min": -5.0,
                                 "v_max": 20.0,
                                 "width": 0.31,
                                 "length": 0.58}
        velocity_gain: float = self.get_parameter('velocity_gain').value

        self.n: int = 32
        self.prediction_horizont: float = 0.6
        self.trajectory_points: int = 10
        self.trajectory_time_difference: float = self.prediction_horizont / self.trajectory_points
        
        self.minimum_velocity: float = 0 
        self.maximum_velocity: float = self.parameters["v_max"] * velocity_gain
        self.minimum_steering_angle: float = - np.pi
        self.maximum_steering_angle: float = np.pi
        
        self.maximum_velocity_difference: float = 10
        self.maximum_steering_difference: float = np.pi / 2

        agent_namespace: str = self.get_parameter('agent_namespace').value
        predictions_topic: str = self.get_parameter('predictions_topic').value

        self.predictions_publisher: rclpy.publisher.Publisher = self.create_publisher(sensor_msgs.PointCloud2, f'{agent_namespace}/{predictions_topic}', 1)


        self.evaluation_client: rclpy.client.Client = self.create_client(EvaluateTrajectories, 'evaluate_trajectories')
        self.evaluation_client.wait_for_service()


    def evaluate(self, trajectories: list[Trajectory]) -> list[float]:
        future = self.evaluation_client.call_async(EvaluateTrajectories.Request(trajectories = trajectories))
        self.executor.spin_until_future_complete(future)
        return future.result().values


    def _convert_state(self, state: list[float]) -> State:
        state = np.array(state, dtype=np.float64)
        return State(state)


    def _sample(self, state: State):   
        g = int(np.sqrt(self.n))
        maximum_steering_difference = self.maximum_steering_difference / (state.velocity**2) if state.velocity > 1 else self.maximum_steering_difference
        steering_angles = np.linspace(self.minimum_steering_angle, 
                                      self.maximum_steering_angle,
                                      g)
        velocities = np.linspace(self.minimum_velocity, 
                                 self.maximum_velocity,
                                 g)

        samples = []
        for s in steering_angles:
            for v in velocities:
                samples.append([s, v])
        return np.array(samples)
 

    def _integrate_state(self, state: State, control: np.ndarray) -> list[Trajectory]:
        trajectories = [Trajectory()] * control.shape[0]

        for i in range(control.shape[0]):
            s = state.internal_state.copy()

            poses: list[Pose2D] = []
            poses.append(conversions.array_to_pose(s[:3]))

            for _ in range(self.trajectory_points):
                n = integrate_state(vehicle_dynamics, s, control[i], self.trajectory_time_difference, self.parameters)
                poses.append(conversions.array_to_pose(n[:3]))
                s = n
            
            trajectories[i].poses = poses
        
        return trajectories


    def plan(self, state: list[float]) -> list[float]:
        state = self._convert_state(state)
   
        control_samples = self._sample(state)

        trajectories = self._integrate_state(state, control_samples)
        values = self.evaluate(trajectories)
        
        best = np.argmin(values)

        self.publish_predictions(trajectories)

        return control_samples[best]
    

    def publish_predictions(self, trajectories: list[Trajectory]):
        if trajectories is None:
            return
        points = []

        trajectory: Trajectory
        for c, trajectory in enumerate(trajectories):
            pose: Pose2D
            for pose in trajectory.poses:
                points.append(np.array([pose.x, pose.y, 0.1, c]))

        points = np.array(points)
        ros_dtype = sensor_msgs.PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize 
        data = points.astype(dtype).tobytes() 

        # The fields specify what the bytes represents. The first 4 bytes 
        # represents the x-coordinate, the next 4 the y-coordinate, etc.
        fields = [sensor_msgs.PointField(name=n, offset=i*itemsize, datatype=ros_dtype, count=1) for i, n in enumerate('xyzc')]

        # The PointCloud2 message also has a header which specifies which 
        # coordinate frame it is represented in. 
        header = std_msgs.Header()
        header.frame_id = 'map'

        msg = sensor_msgs.PointCloud2(
            header=header,
            height=1, 
            width=points.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=(itemsize * 4), # Every point consists of three float32s.
            row_step=(itemsize * 4 * points.shape[0]),
            data=data
        )
        self.predictions_publisher.publish(msg)
    

def main():
    rclpy.init()
    agent = SamplingPlanner()
    rclpy.spin(agent)