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
                 n: int = 64) -> None:
        self.parameters: dict = parameters
        self.centerline: np.ndarray = np.stack([centerline.xs, centerline.ys], dtype=np.float64).T
        self.raceline: np.ndarray = np.stack([raceline.xs, raceline.ys], dtype=np.float64).T
        self.interpolation_factor: float = 0.05

        self.evaluator = LineEvaluation(self.centerline, self.raceline)
    
        assert n > 0
        self.n: int = n
        self.dt: float = 1 / 60
        self.prediction_horizont: float = 0.7
        self.lookahead_distance: float = 4
        
        self.minimum_velocity: float = 0 
        self.maximum_velocity: float = self.parameters["v_max"]
        self.minimum_steering_angle: float = np.pi / 2
        self.maximum_steering_angle: float = -np.pi / 2
        
        self.maximum_velocity_difference: float = self.parameters["a_max"] * self.prediction_horizont
        self.maximum_steering_difference: float = self.parameters["sv_max"] * self.prediction_horizont

        
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

    def _sample(self, state: State, waypoint: np.ndarray):     
        g = int(np.floor(np.sqrt(self.n)))
        direction_to_waypoint = np.arctan2(waypoint[1] - state.position[1], waypoint[0] - state.position[0] + 1e-10)
        direction_to_waypoint= direction_to_waypoint % (2 * np.pi)
        angle_difference = direction_to_waypoint - state.theta

        if angle_difference > 4:
            angle_difference = angle_difference - 2 * np.pi

        elif angle_difference <- 4:
            angle_difference = angle_difference + 2 * np.pi

        steer_cmd = np.arctan2(2.0 * self.wheel_base * np.sin(angle_difference) / self.lookahead_distance, 1.0)
        
        minimum_steering_angle = max(state.steering_angle - self.maximum_steering_difference, self.minimum_steering_angle)
        maximum_stering_angle = min(state.steering_angle + self.maximum_steering_difference, self.maximum_steering_angle)
        
        steering_angles = np.linspace(minimum_steering_angle, maximum_stering_angle, g // 2, dtype=np.float64)
        steering_angles = np.append(steering_angles, np.array([steer_cmd, state.steering_angle, 0]))

        minimum_velocity = max(state.velocity - self.maximum_velocity_difference, self.minimum_velocity)
        maximum_velocity = min(state.velocity + self.maximum_steering_difference, self.maximum_velocity)

        velocities = np.linspace(minimum_velocity, maximum_velocity, g // 2, dtype=np.float64)
        velocities = np.append(velocities, state.velocity)

        samples = np.stack(np.meshgrid(steering_angles, velocities), axis=2)
        samples = np.reshape(samples, (-1, 2))
        return samples, angle_difference
 
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
                         previous_oponent_state: State,
                         waypoint: np.ndarray,
                         angle_difference: float,
                         is_overtaking = False) -> np.ndarray:
        evaluation = np.zeros(len(states))
        for i in range(len(states)):
            evaluation[i] = self.evaluator.evaluate(states[i],
                                                    previous_state,
                                                    oponent_state,
                                                    previous_oponent_state,
                                                    waypoint,
                                                    angle_difference,
                                                    is_overtaking)
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
    
    def plan(self, state: dict, oponent_state: dict, vgain = 1) -> np.ndarray:
        state = self._convert_state(state)
        oponent_state = self._convert_state(oponent_state)
        next_oponent_position =integrate_state(vehicle_dynamics, oponent_state.internal_state, np.zeros(2), self.prediction_horizont, self.parameters)
        next_oponent_state = State(next_oponent_position, self.centerline, self.raceline)
        
        waypoint_on_raceline, _, _ = first_point_on_trajectory_intersecting_circle(state.position, self.lookahead_distance, self.raceline, state.index_on_raceline, True)
        waypoint_on_centerline, _, _ = first_point_on_trajectory_intersecting_circle(state.position, self.lookahead_distance, self.centerline, state.index_on_centerline, True)

        distance_from_oponent = np.linalg.norm(state.position - oponent_state.position)
        oponent_is_leading = oponent_state.index_on_centerline >= state.index_on_centerline - 2
        oponent_is_behind = oponent_state.index_on_centerline < state.index_on_centerline - 2
        is_overtaking = False
        if distance_from_oponent < 3 and vgain == 1:
            is_overtaking = True
            self.overtake_points, self.overtake_waypoint = self._plan_overtake(state, oponent_state, 0.8)
        else:
            self.interpolation_factor = 1
            self.overtake_points = None
            self.overtake_waypoint = None

        if self.overtake_waypoint is not None:
            distance_from_waypoint = np.linalg.norm(self.overtake_waypoint - state.position)
            if oponent_is_leading and distance_from_waypoint > 1.5:
                direction = self.overtake_waypoint - state.position
                direction /= np.linalg.norm(direction)
                multiplier = 3 if distance_from_waypoint < 1.5 else 1.2
                waypoint = self.overtake_waypoint + direction * multiplier
            elif oponent_is_leading:
                self.interpolation_factor = 0.05
                idx = min(state.index_on_centerline + 10, self.centerline.shape[0] - 1)
                direction = self.centerline[idx] - self.centerline[state.index_on_centerline]
                direction /= np.linalg.norm(direction)
                waypoint = state.position + direction * 4
            elif oponent_is_behind and distance_from_waypoint < 2:
                idx = min(state.index_on_centerline + 10, self.centerline.shape[0] - 1)
                direction = self.centerline[idx] - self.centerline[state.index_on_centerline]
                direction /= np.linalg.norm(direction)
                waypoint1 = state.position + direction * 4
                target_waypoint = waypoint_on_centerline + (waypoint_on_raceline - waypoint_on_centerline) / 2
                waypoint = waypoint1 * (1 - self.interpolation_factor) + target_waypoint * self.interpolation_factor
                if (self.interpolation_factor<0.95):
                    self.interpolation_factor += 0.01 
            else:
                is_overtaking = False
                waypoint = waypoint_on_centerline + (waypoint_on_raceline - waypoint_on_centerline) / 2
        else:
            is_overtaking = False
            waypoint = waypoint_on_centerline + (waypoint_on_raceline - waypoint_on_centerline) / 2

        self.waypoint = waypoint
        
        control_samples, angle_difference = self._sample(state, waypoint)
        
        new_states = self._integrate_state(state, control_samples)
        values = self._evaluate_states(new_states, state, next_oponent_state, oponent_state, waypoint ,angle_difference, is_overtaking)
        
        self.next_points = np.array([s.position for s in new_states])
        best = values.argmax()
        self.lookahead_point = self.next_points[best]


        if vgain != 1:
            return [control_samples[best][0], control_samples[best][1] * vgain], new_states

        return control_samples[best], new_states