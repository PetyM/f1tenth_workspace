import rclpy
import rclpy.logging
import rclpy.publisher
import rclpy.subscription
import rclpy.time
from rclpy.node import Node
from geometry_msgs.msg import Transform
from ackermann_msgs.msg import AckermannDriveStamped
from transforms3d import euler
import numpy as np
from samplingplanner import SamplingPlanner
from std_msgs.msg import Float64MultiArray
from f1tenth_gym.envs.track import Raceline
import pathlib

def transform_to_array(t: Transform) -> np.ndarray:
    e = euler.quat2euler([t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z])
    return np.concatenate([np.array([t.translation.x, t.translation.y]), e])


class Agent(Node):
    def __init__(self):
        super().__init__('agent')
        self.get_logger().info('Agent.init')

        self.declare_parameter('agent_namespace', 'ego_racecar')
        self.declare_parameter('opponent_namespace', 'opp_racecar')
        self.declare_parameter('state_topic', 'state')
        self.declare_parameter('drive_topic', 'drive')
        self.declare_parameter('velocity_gain', 1.0)

        self.declare_parameter('map_name', '')
        self.declare_parameter('map_folder_path', '')

        self.ego_state: list[float] = None
        self.opp_state: list[float] = None

        agent_namespace: str = self.get_parameter('agent_namespace').value
        opponent_namespace: str = self.get_parameter('opponent_namespace').value
        drive_topic: str = self.get_parameter('drive_topic').value
        state_topic: str = self.get_parameter('state_topic').value

        self.drive_publiser: rclpy.publisher.Publisher = self.create_publisher(AckermannDriveStamped, f'{agent_namespace}/{drive_topic}', 1)

        self.ego_state_subscriber: rclpy.subscription.Subscription = self.create_subscription(Float64MultiArray, f'{agent_namespace}/{state_topic}', self.ego_state_cb, 10)
        self.opp_state_subscriber: rclpy.subscription.Subscription = self.create_subscription(Float64MultiArray, f'{opponent_namespace}/{state_topic}', self.opp_state_cb, 10)

        self.velocity_gain: float = self.get_parameter('velocity_gain').value

        params = {"mu": 1.0489, 
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
        
        map_name = self.get_parameter('map_name').value
        map_folder_path = self.get_parameter('map_folder_path').value

        centerline_file = pathlib.Path(f"{map_folder_path}/{map_name}_centerline.csv")
        raceline_file = pathlib.Path(f"{map_folder_path}/{map_name}_raceline.csv")

        self.planner: SamplingPlanner = SamplingPlanner(params, Raceline.from_centerline_file(centerline_file), Raceline.from_raceline_file(raceline_file))

    def ego_state_cb(self, msg: Float64MultiArray):
        self.ego_state = msg.data

    def opp_state_cb(self, msg: Float64MultiArray):
        self.opp_state = msg.data

    def update(self):
        if self.ego_state and self.opp_state:
            action = self.planner.plan(self.ego_state, self.opp_state, self.velocity_gain)

            msg = AckermannDriveStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.drive.speed = action[0]
            msg.drive.steering_angle = action[1]
            self.drive_publiser.publish(msg)


def main():
    rclpy.init()
    agent = Agent()
    
    while rclpy.ok():
        rclpy.spin_once(agent)
        agent.update()
