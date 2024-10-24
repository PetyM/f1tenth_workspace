import numpy as np
from state import State

class LineEvaluation:
    def __init__(self,
                 centerline: np.ndarray,
                 raceline: np.ndarray) -> None:
        self.centerline: np.ndarray = centerline
        self.raceline: np.ndarray = raceline

    
    def evaluate(self,
                 state: State, 
                 previous_state: State,
                 oponent_state: State, 
                 previous_oponent_state: State) -> float:
        traveled_distance = np.linalg.norm(state.position - previous_state.position)

        # Prefer stable steering inputs
        steering_diff_multiplier = 0
        if state.velocity > 4:
            steering_diff_multiplier = 0.2
        steering_diff = np.abs(previous_state.steering_angle - state.steering_angle)
        
        # keeps car in between centerline and raceline  
        projections_distance = np.linalg.norm(state.position_on_centerline - state.position_on_raceline)
        raceline_factor = ((projections_distance / 2) - state.distance_from_raceline)**2
        centerline_factor = ((projections_distance / 2) - state.distance_from_centerline)**2
        
        # if oponent is near, prefer states further from oponent
        distance_from_oponent = np.linalg.norm(state.position - previous_oponent_state.position)
        oponent_factor = distance_from_oponent if np.linalg.norm(previous_state.position - previous_oponent_state.position) < 2 else 0
        traveled_distance/=1.5
        # if(np.abs(previous_state.steering_angle - state.steering_angle)>0.2):
        #     return -100
   
        if np.linalg.norm(state.position - oponent_state.position) < 0.3 or state.distance_from_centerline>0.8 or np.linalg.norm(previous_state.position - previous_oponent_state.position) < 0.2:
            evaluation = traveled_distance - raceline_factor - centerline_factor - 1000
            return evaluation
            
        # Do not pick states that are fully stopped if there is no oponent
        # if oponent_factor == 0 and state.velocity < 0.2:
        #     return -100
        
        evaluation = traveled_distance - raceline_factor - centerline_factor - steering_diff * steering_diff_multiplier
        
        return evaluation