import numpy as np
from f1tenth_gym.envs.track import Raceline
from model import vehicle_dynamics, integrate_state

from lineevaluator import LineEvaluation
from state import State

from geometry_msgs.msg import Pose2D
from interfaces.msg import Trajectory

import conversions

import typing
from typing import Callable


class SamplingPlanner:
           
    def __init__(self,
                 parameters: dict,
                 velocity_gain: float,
                 evaluate_function: Callable[[list[Trajectory]], list[float]]) -> None:

        self.parameters: dict = parameters
        self.interpolation_factor: float = 0.05

        self.evaluate_function: Callable[[list[Trajectory]], list[float]] = evaluate_function

        self.n: int = 32
        self.prediction_horizont: float = 0.6
        self.trajectory_points: int = 10
        self.trajectory_time_difference: float = self.prediction_horizont / self.trajectory_points
        
        self.minimum_velocity: float = 0 
        self.maximum_velocity: float = self.parameters["v_max"] * velocity_gain
        self.minimum_steering_angle: float = - np.pi
        self.maximum_steering_angle: float = np.pi
        
        self.maximum_velocity_difference: float = 0.5
        self.maximum_steering_difference: float = np.pi / 2


    def _convert_state(self, state: list[float]) -> State:
        state = np.array(state, dtype=np.float64)
        return State(state)


    def _sample(self, state: State):   
        g = int(np.sqrt(self.n))
        maximum_steering_difference = self.maximum_steering_difference / (state.velocity**2) if state.velocity > 1 else self.maximum_steering_difference
        steering_angles = np.linspace(max(state.steering_angle - maximum_steering_difference, self.minimum_steering_angle), 
                                      min(state.steering_angle + maximum_steering_difference, self.maximum_steering_angle),
                                      g)
        velocities = np.linspace(max(state.velocity - self.maximum_velocity_difference, self.minimum_velocity), 
                                 min(state.velocity + self.maximum_velocity_difference, self.maximum_velocity),
                                 g)

        samples = np.stack(np.meshgrid(steering_angles, velocities), axis=2)
        samples = np.reshape(samples, (-1, 2))
        return samples
 

    def _integrate_state(self, state: State, control: np.ndarray) -> list[Trajectory]:
        trajectories = [Trajectory()] * control.shape[0]
        for i in range(control.shape[0]):
            s = state.internal_state
            poses: list[Pose2D] = []
            for _ in range(self.trajectory_points):
                s = integrate_state(vehicle_dynamics, s, control[i], self.trajectory_time_difference, self.parameters)
                poses.append(conversions.array_to_pose(s[:3]))
            trajectories[i].poses = poses
        return trajectories


    def plan(self, state: dict) -> np.ndarray:
        state = self._convert_state(state)
   
        control_samples = self._sample(state)

        trajectories = self._integrate_state(state, control_samples)
        values = self.evaluate_function(trajectories)
        
        best = np.argmax(values)

        return control_samples[best], trajectories