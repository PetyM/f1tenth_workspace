import numpy as np
from f1tenth_gym.envs.track import Raceline
from model import vehicle_dynamics, integrate_state

from lineevaluator import LineEvaluation
from state import State, first_point_on_trajectory_intersecting_circle

np.set_printoptions(threshold=np.inf)
np.set_printoptions(linewidth=np.inf)

class SamplingPlanner:
    def __init__(self,
                 parameters: dict,
                 centerline: Raceline,
                 raceline: Raceline,
                 velocity_gain: float) -> None:
        self.parameters: dict = parameters
        self.centerline: np.ndarray = np.stack([centerline.xs, centerline.ys], dtype=np.float64).T
        self.raceline: np.ndarray = np.stack([raceline.xs, raceline.ys], dtype=np.float64).T
        self.interpolation_factor: float = 0.05

        self.evaluator = LineEvaluation(self.centerline, self.raceline)
    
        self.n: int = 32
        self.dt: float = 1 / 60
        self.prediction_horizont: float = 0.6
        self.lookahead_distance: float = 4
        
        self.minimum_velocity: float = 0 
        self.maximum_velocity: float = self.parameters["v_max"] * velocity_gain
        self.minimum_steering_angle: float = - np.pi
        self.maximum_steering_angle: float = np.pi
        
        self.maximum_velocity_difference: float = 0.5
        self.maximum_steering_difference: float = np.pi / 2

        
        self.future_steps: int = int(self.prediction_horizont / self.dt)
        self.wheel_base: float = self.parameters["lf"] + self.parameters["lr"]
        
        self.next_points: np.ndarray = None
        self.overtake_points: np.ndarray = None
        self.lookahead_point: np.ndarray = None
        self.waypoint: np.ndarray = None
        self.overtake_waypoint: np.ndarray = None

    def _convert_state(self, state: list[float]) -> State:
        state = np.array(state, dtype=np.float64)
        return State(state, self.centerline, self.raceline)

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
 
    def _integrate_state(self, state: State, control: np.ndarray) -> list[State]:
        new_states = [None] * control.shape[0]
        for i in range(control.shape[0]):
            s = state.internal_state
            new_states[i] = State(integrate_state(vehicle_dynamics, s, control[i], self.prediction_horizont, self.parameters), self.centerline, self.raceline)
        return new_states

    def _evaluate_states(self,
                         states: list[State],
                         previous_state: State,
                         oponent_state: State,
                         previous_oponent_state: State) -> np.ndarray:
        evaluation = np.zeros(len(states))
        for i in range(len(states)):
            evaluation[i] = self.evaluator.evaluate(states[i],
                                                    previous_state,
                                                    oponent_state,
                                                    previous_oponent_state)
        return evaluation
    
    def _plan_overtake(self, state: State, oponent_state: State, radius = 1):
        start_i = max(state.index_on_centerline - 10, 0)
        end_i = min(start_i + self.lookahead_distance * 15, self.centerline.shape[0] - 1)
        overtake_path = self.centerline[start_i:end_i,:]

        distances = np.linalg.norm(overtake_path - oponent_state.position, axis=1)    
        within_radius = distances < radius

        if not np.any(within_radius):
            return overtake_path, None

        adjusted = overtake_path.copy()
        for i, point in enumerate(adjusted):
            if within_radius[i]:
                direction = point - oponent_state.position
                direction /= np.linalg.norm(direction)
                adjusted[i] += direction * (radius - distances[i])

        waypoint_i = np.argmin(distances)
        
        return adjusted, adjusted[waypoint_i]
    
    def plan(self, state: dict, oponent_state: dict) -> np.ndarray:
        state = self._convert_state(state)
        oponent_state = self._convert_state(oponent_state)
        next_oponent_position = integrate_state(vehicle_dynamics, oponent_state.internal_state, np.zeros(2), self.prediction_horizont, self.parameters)
        next_oponent_state = State(next_oponent_position, self.centerline, self.raceline)
                
        control_samples = self._sample(state)
        
        new_states = self._integrate_state(state, control_samples)
        values = self._evaluate_states(new_states, state, next_oponent_state, oponent_state)
        
        self.next_points = np.array([s.position for s in new_states])
        best = values.argmax()

        return control_samples[best], new_states