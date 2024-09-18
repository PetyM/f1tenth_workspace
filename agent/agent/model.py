import numpy as np
from numba import njit

@njit(cache=True)
def accl_constraints(vel, accl, v_switch, a_max, v_min, v_max):
    """
    Acceleration constraints, adjusts the acceleration based on constraints

        Args:
            vel (float): current velocity of the vehicle
            accl (float): unconstraint desired acceleration
            v_switch (float): switching velocity (velocity at which the acceleration is no longer able to create wheel spin)
            a_max (float): maximum allowed acceleration
            v_min (float): minimum allowed velocity
            v_max (float): maximum allowed velocity

        Returns:
            accl (float): adjusted acceleration
    """

    # positive accl limit
    if vel > v_switch:
        pos_limit = a_max*v_switch/vel
    else:
        pos_limit = a_max

    # accl limit reached?
    if (vel <= v_min and accl <= 0) or (vel >= v_max and accl >= 0):
        accl = 0.
    elif accl <= -a_max:
        accl = -a_max
    elif accl >= pos_limit:
        accl = pos_limit

    return accl

@njit(cache=True)
def steering_constraint(steering_angle, steering_velocity, s_min, s_max, sv_min, sv_max):
    """
    Steering constraints, adjusts the steering velocity based on constraints

        Args:
            steering_angle (float): current steering_angle of the vehicle
            steering_velocity (float): unconstraint desired steering_velocity
            s_min (float): minimum steering angle
            s_max (float): maximum steering angle
            sv_min (float): minimum steering velocity
            sv_max (float): maximum steering velocity

        Returns:
            steering_velocity (float): adjusted steering velocity
    """

    # constraint steering velocity
    if (steering_angle <= s_min and steering_velocity <= 0) or (steering_angle >= s_max and steering_velocity >= 0):
        steering_velocity = 0.
    elif steering_velocity <= sv_min:
        steering_velocity = sv_min
    elif steering_velocity >= sv_max:
        steering_velocity = sv_max

    return steering_velocity


def vehicle_dynamics(x, u_init, lf, lr, s_min, s_max, sv_min, sv_max, v_switch, a_max, v_min, v_max):
    # constraints
    u = np.array([steering_constraint(x[2], u_init[0], s_min, s_max, sv_min, sv_max), accl_constraints(x[3], u_init[1], v_switch, a_max, v_min, v_max)])

    # wheelbase
    lwb = lf + lr

    f = np.array([x[3] * np.cos(x[4]), 
                  x[3] * np.sin(x[4]), 
                  u[0], 
                  u[1], 
                  x[3] / lwb * np.tan(x[2]), 
                  u[1] / lwb * np.tan(x[2]) + x[3] / (lwb * np.cos(x[2])**2) * u[0],
                  0])
    return f


def integrate_state(f, x, u, dt, params):
    k1 = f(
        x,
        u,
        params["mu"],
        params["C_Sf"],
        params["C_Sr"],
        params["lf"],
        params["lr"],
        params["h"],
        params["m"],
        params["I"],
        params["s_min"],
        params["s_max"],
        params["sv_min"],
        params["sv_max"],
        params["v_switch"],
        params["a_max"],
        params["v_min"],
        params["v_max"],
    )

    k2_state = x + dt * (k1 / 2)

    k2 = f(
        k2_state,
        u,
        params["mu"],
        params["C_Sf"],
        params["C_Sr"],
        params["lf"],
        params["lr"],
        params["h"],
        params["m"],
        params["I"],
        params["s_min"],
        params["s_max"],
        params["sv_min"],
        params["sv_max"],
        params["v_switch"],
        params["a_max"],
        params["v_min"],
        params["v_max"],
    )

    k3_state = x + dt * (k2 / 2)

    k3 = f(
        k3_state,
        u,
        params["mu"],
        params["C_Sf"],
        params["C_Sr"],
        params["lf"],
        params["lr"],
        params["h"],
        params["m"],
        params["I"],
        params["s_min"],
        params["s_max"],
        params["sv_min"],
        params["sv_max"],
        params["v_switch"],
        params["a_max"],
        params["v_min"],
        params["v_max"],
    )

    k4_state = x + dt * k3

    k4 = f(
        k4_state,
        u,
        params["mu"],
        params["C_Sf"],
        params["C_Sr"],
        params["lf"],
        params["lr"],
        params["h"],
        params["m"],
        params["I"],
        params["s_min"],
        params["s_max"],
        params["sv_min"],
        params["sv_max"],
        params["v_switch"],
        params["a_max"],
        params["v_min"],
        params["v_max"],
    )

    # dynamics integration
    x = x + dt * (1 / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
    return x