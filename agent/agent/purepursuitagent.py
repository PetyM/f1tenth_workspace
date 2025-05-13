import rclpy
import rclpy.callback_groups
import rclpy.client
import rclpy.logging
import rclpy.node
import rclpy.publisher
import rclpy.subscription
import rclpy.time
import rclpy.timer
import rclpy.qos

from ackermann_msgs.msg import AckermannDriveStamped

from std_msgs.msg import Float64MultiArray, Bool
from std_srvs.srv import Empty
import time
from typing import Tuple

import gymnasium as gym
import numpy as np
from numba import njit
from agent.agentbase import AgentBase

import pathlib
from f1tenth_gym.envs.track import Raceline
from f1tenth_gym.envs.dynamic_models import pid_steer, pid_accl

from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan

from std_msgs.msg import Float64MultiArray, Bool
from std_srvs.srv import Empty
from f1tenth_gym.envs.dynamic_models import pid_steer, pid_accl


@njit(fastmath=False, cache=True)
def nearest_point_on_trajectory(point, trajectory):
    """
    Return the nearest point along the given piecewise linear trajectory.

    Same as nearest_point_on_line_segment, but vectorized. This method is quite fast, time constraints should
    not be an issue so long as trajectories are not insanely long.

        Order of magnitude: trajectory length: 1000 --> 0.0002 second computation (5000fps)

    point: size 2 numpy array
    trajectory: Nx2 matrix of (x,y) trajectory waypoints
        - these must be unique. If they are not unique, a divide by 0 error will destroy the world
    """
    diffs = trajectory[1:, :] - trajectory[:-1, :]
    l2s = diffs[:, 0] ** 2 + diffs[:, 1] ** 2
    # this is equivalent to the elementwise dot product
    # dots = np.sum((point - trajectory[:-1,:]) * diffs[:,:], axis=1)
    dots = np.empty((trajectory.shape[0] - 1,))
    for i in range(dots.shape[0]):
        dots[i] = np.dot((point - trajectory[i, :]), diffs[i, :])
    t = dots / l2s
    t[t < 0.0] = 0.0
    t[t > 1.0] = 1.0
    # t = np.clip(dots / l2s, 0.0, 1.0)
    projections = trajectory[:-1, :] + (t * diffs.T).T
    # dists = np.linalg.norm(point - projections, axis=1)
    dists = np.empty((projections.shape[0],))
    for i in range(dists.shape[0]):
        temp = point - projections[i]
        dists[i] = np.sqrt(np.sum(temp * temp))
    min_dist_segment = np.argmin(dists)
    return (
        projections[min_dist_segment],
        dists[min_dist_segment],
        t[min_dist_segment],
        min_dist_segment,
    )


@njit(fastmath=False, cache=True)
def first_point_on_trajectory_intersecting_circle(
    point, radius, trajectory, t=0.0, wrap=False
):
    """
    starts at beginning of trajectory, and find the first point one radius away from the given point along the trajectory.

    Assumes that the first segment passes within a single radius of the point

    http://codereview.stackexchange.com/questions/86421/line-segment-to-circle-collision-algorithm
    """
    start_i = int(t)
    start_t = t % 1.0
    first_t = None
    first_i = None
    first_p = None
    trajectory = np.ascontiguousarray(trajectory)
    for i in range(start_i, trajectory.shape[0] - 1):
        start = trajectory[i, :]
        end = trajectory[i + 1, :] + 1e-6
        V = np.ascontiguousarray(end - start).astype(
            np.float32
        )  # NOTE: specify type or numba complains

        a = np.dot(V, V)
        b = np.float32(2.0) * np.dot(
            V, start - point
        )  # NOTE: specify type or numba complains
        c = (
            np.dot(start, start)
            + np.dot(point, point)
            - np.float32(2.0) * np.dot(start, point)
            - radius * radius
        )
        discriminant = b * b - 4 * a * c

        if discriminant < 0:
            continue
        #   print "NO INTERSECTION"
        # else:
        # if discriminant >= 0.0:
        discriminant = np.sqrt(discriminant)
        t1 = (-b - discriminant) / (2.0 * a)
        t2 = (-b + discriminant) / (2.0 * a)
        if i == start_i:
            if t1 >= 0.0 and t1 <= 1.0 and t1 >= start_t:
                first_t = t1
                first_i = i
                first_p = start + t1 * V
                break
            if t2 >= 0.0 and t2 <= 1.0 and t2 >= start_t:
                first_t = t2
                first_i = i
                first_p = start + t2 * V
                break
        elif t1 >= 0.0 and t1 <= 1.0:
            first_t = t1
            first_i = i
            first_p = start + t1 * V
            break
        elif t2 >= 0.0 and t2 <= 1.0:
            first_t = t2
            first_i = i
            first_p = start + t2 * V
            break
    # wrap around to the beginning of the trajectory if no intersection is found1
    if wrap and first_p is None:
        for i in range(-1, start_i):
            start = trajectory[i % trajectory.shape[0], :]
            end = trajectory[(i + 1) % trajectory.shape[0], :] + 1e-6
            V = (end - start).astype(np.float32)

            a = np.dot(V, V)
            b = np.float32(2.0) * np.dot(
                V, start - point
            )  # NOTE: specify type or numba complains
            c = (
                np.dot(start, start)
                + np.dot(point, point)
                - np.float32(2.0) * np.dot(start, point)
                - radius * radius
            )
            discriminant = b * b - 4 * a * c

            if discriminant < 0:
                continue
            discriminant = np.sqrt(discriminant)
            t1 = (-b - discriminant) / (2.0 * a)
            t2 = (-b + discriminant) / (2.0 * a)
            if t1 >= 0.0 and t1 <= 1.0:
                first_t = t1
                first_i = i
                first_p = start + t1 * V
                break
            elif t2 >= 0.0 and t2 <= 1.0:
                first_t = t2
                first_i = i
                first_p = start + t2 * V
                break

    return first_p, first_i, first_t


@njit(fastmath=False, cache=True)
def get_actuation(pose_theta, lookahead_point, position, lookahead_distance, wheelbase):
    """
    Returns actuation
    """
    waypoint_y = np.dot(
        np.array([np.sin(-pose_theta), np.cos(-pose_theta)], dtype=np.float32),
        lookahead_point[0:2] - position,
    )
    speed = lookahead_point[2]
    if np.abs(waypoint_y) < 1e-6:
        return speed, 0.0
    radius = 1 / (2.0 * waypoint_y / lookahead_distance**2)
    steering_angle = np.arctan(wheelbase / radius)
    return speed, steering_angle


class PurePursuitAgent(rclpy.node.Node):
    def __init__(self):
        super().__init__('agent')

        self.declare_parameter('agent_namespace', 'ego_racecar')
        self.agent_namespace: str = self.get_parameter('agent_namespace').value

        self.declare_parameter('map_name', '')
        self.map_name: str = self.get_parameter('map_name').value

        self.declare_parameter('map_folder_path', '')
        self.map_folder_path: str = self.get_parameter('map_folder_path').value

        self.declare_parameter('velocity_gain', 1.0)
        self.velocity_gain: float =  self.get_parameter('velocity_gain').value

        self.params = {"mu": 1.0489, 
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
        self.wheelbase = self.params["lr"] + self.params["lf"]
        centerline_file = pathlib.Path(f"{self.map_folder_path}/{self.map_name}_raceline.csv")


        centerline = Raceline.from_raceline_file(centerline_file)
        self.waypoints = np.stack([centerline.xs, centerline.ys, centerline.vxs]).T
        self.max_reacquire = 20.0
        self.lookahead_distance = 0.82461887897713965

        self.ego_state_subscriber: rclpy.subscription.Subscription = self.create_subscription(Float64MultiArray, f'{self.agent_namespace}/state', self.ego_state_cb, 10)
        self.drive_publiser: rclpy.publisher.Publisher = self.create_publisher(AckermannDriveStamped, f'{self.agent_namespace}/drive', 1)
        self.timer_update_control: rclpy.timer.Timer = self.create_timer(0.001, self.update_control)
        self.target_speed: float = 0.0
        self.target_steering_angle: float = 0.0
        self.ego_state: list[float] = [0,0,0,0,0,0,0]

        self.ready()

    def ego_state_cb(self, msg: Float64MultiArray):
        self.ego_state = msg.data

        pose_x = np.float32(self.ego_state[0])
        pose_y = np.float32(self.ego_state[1])
        pose_theta = np.float32(self.ego_state[4])
        position = np.array([pose_x, pose_y])
        lookahead_point, _ = self._get_current_waypoint(self.waypoints, self.lookahead_distance, position, pose_theta)

        if lookahead_point is None:
            return [4.0, 0.0]

        # actuation
        speed, steering_angle = get_actuation(pose_theta, lookahead_point, position, self.lookahead_distance, self.wheelbase)

        self.target_speed = self.velocity_gain * speed
        self.target_steering_angle = steering_angle


    def ready(self):
        client = self.create_client(Empty, f'{self.agent_namespace}/ready')
        client.wait_for_service()
        client.call_async(Empty.Request())
        self.get_logger().info('Ready')


    def update_control(self):
        acceleration = pid_accl(self.target_speed, self.ego_state[3], self.params["a_max"], self.params["v_max"] * self.velocity_gain, self.params["v_min"],)
        steering = pid_steer(self.target_steering_angle, self.ego_state[2], self.params["sv_max"])
        self.get_logger().info(f'{acceleration=}, {steering=} ')

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.acceleration = float(acceleration)
        msg.drive.steering_angle_velocity = float(steering)
        self.drive_publiser.publish(msg)


    def _get_current_waypoint(
        self, waypoints, lookahead_distance, position, theta
    ) -> Tuple[np.ndarray, int]:
        """
        Returns the current waypoint to follow given the current pose.

        Args:
            waypoints: The waypoints to follow (Nx3 array)
            lookahead_distance: The lookahead distance [m]
            position: The current position (2D array)
            theta: The current heading [rad]

        Returns:
            waypoint: The current waypoint to follow (x, y, speed)
            i: The index of the current waypoint
        """
        wpts = waypoints[:, :2]
        lookahead_distance = np.float32(lookahead_distance)
        nearest_point, nearest_dist, t, i = nearest_point_on_trajectory(position, wpts)
        if nearest_dist < lookahead_distance:
            t1 = np.float32(i + t)
            lookahead_point, i2, t2 = first_point_on_trajectory_intersecting_circle(
                position, lookahead_distance, wpts, t1, wrap=True
            )
            if i2 is None:
                return None, None
            current_waypoint = np.empty((3,), dtype=np.float32)
            # x, y
            current_waypoint[0:2] = wpts[i2, :]
            # speed
            current_waypoint[2] = waypoints[i, -1]
            return current_waypoint, i
        elif nearest_dist < self.max_reacquire:
            # NOTE: specify type or numba complains
            return wpts[i, :], i
        else:
            return None, None


def main():
    rclpy.init()
    agent = PurePursuitAgent()
    rclpy.spin(agent)