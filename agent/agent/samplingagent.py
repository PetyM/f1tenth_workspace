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
from custom_interfaces.msg import Trajectory

import conversions

from agent.agent import Agent

from geometry_msgs.msg import Pose2D
from custom_interfaces.srv import EvaluateTrajectories
from custom_interfaces.msg import Trajectory, TrajectoryEvaluation
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs

class SamplingAgent(Agent):
    def __init__(self) -> None:
        super().__init__()
    
        self.declare_parameter('predictions_topic', 'predictions')
        self.declare_parameter('followed_trajectory_topic', 'followed_trajectory')
        self.predictions_topic: str = self.get_parameter('predictions_topic').value
        self.followed_trajectory_topic: str = self.get_parameter('followed_trajectory_topic').value

        self.predictions_publisher: rclpy.publisher.Publisher = self.create_publisher(sensor_msgs.PointCloud2, f'{self.agent_namespace}/{self.predictions_topic}', 1)
        self.followed_trajectory_publisher: rclpy.publisher.Publisher = self.create_publisher(sensor_msgs.PointCloud2, f'{self.agent_namespace}/{self.followed_trajectory_topic}', 1)

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

        self.steering_saples_count: int = 5
        self.velocity_samples_count: int = 3
        self.prediction_horizont: float = 2.0
        self.trajectory_points: int = 10
        self.trajectory_time_difference: float = self.prediction_horizont / self.trajectory_points
        
        self.minimum_velocity: float = 0.0
        self.maximum_velocity: float = 0.7 * self.parameters["v_max"] * self.velocity_gain
        self.minimum_steering_angle: float = 0.7 * self.parameters["s_min"]
        self.maximum_steering_angle: float = 0.7 * self.parameters["s_max"]
        
        self.maximum_velocity_difference: float = (self.maximum_velocity - self.minimum_velocity) / 10.0
        self.maximum_steering_difference: float = (self.maximum_steering_angle - self.minimum_steering_angle) / 10.0


    def _convert_state(self, state: list[float]) -> State:
        state = np.array(state, dtype=np.float64)
        return State(state)


    def generate_samples(self, state: State):   
        maximum_steering_difference = self.maximum_steering_difference # / (state.velocity**2) if state.velocity > 1 else self.maximum_steering_difference

        maximum_steering = state.steering_angle + maximum_steering_difference
        minimum_steering = state.steering_angle - maximum_steering_difference

        if maximum_steering > self.maximum_steering_angle:
            maximum_steering = self.maximum_steering_angle
            minimum_steering = max(-self.minimum_steering_angle, maximum_steering - 2 * maximum_steering_difference)
        elif minimum_steering < -self.maximum_steering_angle:
            minimum_steering = -self.minimum_steering_angle
            maximum_steering = min(self.maximum_steering_angle, minimum_steering + 2 * maximum_steering_difference)
        
        steering_angles = np.linspace(minimum_steering, maximum_steering, self.steering_saples_count)
        
        velocities = np.linspace(max(state.velocity - self.maximum_velocity_difference, self.minimum_velocity), 
                                 min(state.velocity + self.maximum_velocity_difference, self.maximum_velocity),
                                 self.velocity_samples_count)

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


    def evaluate_trajectories(self, trajectories: list[Trajectory]) -> list[TrajectoryEvaluation]:
        future = self.evaluation_client.call_async(EvaluateTrajectories.Request(trajectories = trajectories))
        self.executor.spin_until_future_complete(future)
        return future.result().values
    

    def plan(self, state: list[float]) -> list[float]:
        state: State = self._convert_state(state)
   
        control_samples = self.generate_samples(state)

        trajectories = self.generate_trajectories(state, control_samples)

        assert len(control_samples) == len(trajectories), 'Trajectories count doesnt match samples count'

        self.publish_predictions(trajectories)

        trajectories_evaluation = self.evaluate_trajectories(trajectories)
        
        assert len(trajectories) == len(trajectories_evaluation), 'Evaluations count doesnt match trajectories count'

        trajectories_evaluated = []
        control_samples_filtered = []
        trajectories_filtered = []
        i = 0
        for sample, trajectory, evaluation in zip(control_samples, trajectories, trajectories_evaluation):
            if not evaluation.collision:
                trajectories_evaluated.append([i, evaluation.progress, evaluation.trajectory_cost])
                control_samples_filtered.append(sample)
                trajectories_filtered.append(trajectory)
                i += 1

        if len(control_samples_filtered) == 0:
            return [0, 0]
        
        trajectories_by_progress = sorted(trajectories_evaluated, key= lambda x: x[1], reverse=True)
        trajectories_by_cost = sorted(trajectories_evaluated, key= lambda x: x[2])

        progress_scores = np.array([0] * len(trajectories_evaluated))
        cost_scores = np.array([0] * len(trajectories_evaluated))
        for i in range(len(trajectories_evaluated)):
            progress_scores[trajectories_by_progress[i][0]] = i
            cost_scores[trajectories_by_cost[i][0]] = i
    
        relative_velocity = state.velocity / self.maximum_velocity
        cost_factor = 0.8 + 0.5 * (relative_velocity - 0.5)
        combined_scores = progress_scores + cost_factor * cost_scores

        best = np.argmin(combined_scores)

        self.publish_followed_trajectory(trajectories_filtered[best])

        return control_samples_filtered[best]
    

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
    

    def publish_followed_trajectory(self, trajectory: Trajectory):
        if trajectory is None:
            return
        points = []

        pose: Pose2D
        for pose in trajectory.poses:
            points.append(np.array([pose.x, pose.y, 0.1]))

        points = np.array(points)
        ros_dtype = sensor_msgs.PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize 
        data = points.astype(dtype).tobytes() 

        fields = [sensor_msgs.PointField(name=n, offset=i*itemsize, datatype=ros_dtype, count=1) for i, n in enumerate('xyz')]
        header = std_msgs.Header()
        header.frame_id = 'map'

        msg = sensor_msgs.PointCloud2(
            header=header,
            height=1, 
            width=points.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=(itemsize * 3), # Every point consists of 4 float32s.
            row_step=(itemsize * 3 * points.shape[0]),
            data=data
        )
        self.followed_trajectory_publisher.publish(msg)
    

def main():
    rclpy.init()    
    
    qos_profile = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                depth=5)
    rclpy.wait_for_message.wait_for_message(OccupancyGrid, Node('waiter'), '/costmap', qos_profile=qos_profile, time_to_wait=-1)

    agent = SamplingAgent()
    rclpy.spin(agent)