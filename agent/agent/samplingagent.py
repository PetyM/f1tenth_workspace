import rclpy
import rclpy.client
import rclpy.publisher
import rclpy.wait_for_message

from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

import numpy as np
from model import vehicle_dynamics, integrate_state

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

class SamplingAgent(Agent):
    def __init__(self) -> None:
        super().__init__()
    
        self.declare_parameter('predictions_topic', 'predictions')
        self.predictions_topic: str = self.get_parameter('predictions_topic').value

        self.predictions_publisher: rclpy.publisher.Publisher = self.create_publisher(sensor_msgs.PointCloud2, f'{self.agent_namespace}/{self.predictions_topic}', 1)

        self.evaluation_client: rclpy.client.Client = self.create_client(EvaluateTrajectories, 'evaluate_trajectories')
        self.evaluation_client.wait_for_service()
        
        
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

        self.n: int = 3
        self.prediction_horizont: float = 0.6
        self.trajectory_points: int = 10
        self.trajectory_time_difference: float = self.prediction_horizont / self.trajectory_points
        
        self.minimum_velocity: float = 0 
        self.maximum_velocity: float = self.parameters["v_max"] * self.velocity_gain
        self.minimum_steering_angle: float = - np.pi
        self.maximum_steering_angle: float = np.pi
        
        self.maximum_velocity_difference: float = 10
        self.maximum_steering_difference: float = np.pi / 2


    def _convert_state(self, state: list[float]) -> State:
        state = np.array(state, dtype=np.float64)
        return State(state)


    def generate_samples(self, state: State):   
        maximum_steering_difference = self.maximum_steering_difference / (state.velocity**2) if state.velocity > 1 else self.maximum_steering_difference
        steering_angles = np.linspace(max(state.steering_angle - maximum_steering_difference, self.minimum_steering_angle), 
                                      min(state.steering_angle + maximum_steering_difference, self.maximum_steering_angle),
                                      self.n)
        velocities = np.linspace(max(state.velocity - self.maximum_velocity_difference, self.minimum_velocity), 
                                 min(state.velocity + self.maximum_velocity_difference, self.maximum_velocity),
                                 self.n)

        samples = np.stack(np.meshgrid(steering_angles, velocities), axis=2)
        samples = np.reshape(samples, (-1, 2))
        return samples
 

    def generate_trajectories(self, state: State, control: np.ndarray) -> list[Trajectory]:
        trajectories: list[Trajectory] = []

        for i in range(control.shape[0]):
            s = state.internal_state.copy()

            poses: list[Pose2D] = []
            poses.append(conversions.array_to_pose(s[:3]))

            for _ in range(self.trajectory_points):
                s = integrate_state(vehicle_dynamics, s, control[i], self.trajectory_time_difference, self.parameters)
                poses.append(conversions.array_to_pose(s[:3]))

            trajectories.append(Trajectory(poses=poses))
        
        return trajectories


    def evaluate_trajectories(self, trajectories: list[Trajectory]) -> list[float]:
        future = self.evaluation_client.call_async(EvaluateTrajectories.Request(trajectories = trajectories))
        self.executor.spin_until_future_complete(future)
        return future.result().values
    

    def plan(self, state: list[float]) -> list[float]:
        state = self._convert_state(state)
   
        control_samples = self.generate_samples(state)

        trajectories = self.generate_trajectories(state, control_samples)
        values = self.evaluate_trajectories(trajectories)
        
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
            point_step=(itemsize * 4), # Every point consists of 4 float32s.
            row_step=(itemsize * 4 * points.shape[0]),
            data=data
        )
        self.predictions_publisher.publish(msg)
    

def main():
    rclpy.init()    
    
    qos_profile = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                depth=5)
    rclpy.wait_for_message.wait_for_message(OccupancyGrid, Node('waiter'), '/costmap', qos_profile=qos_profile, time_to_wait=-1)

    agent = SamplingAgent()
    rclpy.spin(agent)