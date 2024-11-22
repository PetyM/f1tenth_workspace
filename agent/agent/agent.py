import rclpy
import rclpy.callback_groups
import rclpy.client
import rclpy.logging
import rclpy.publisher
import rclpy.subscription
import rclpy.time
import rclpy.timer
import rclpy.qos

from rclpy.node import Node
from geometry_msgs.msg import Transform
from ackermann_msgs.msg import AckermannDriveStamped
import rclpy.wait_for_message
from transforms3d import euler
import numpy as np

from std_msgs.msg import Float64MultiArray

from nav_msgs.msg import OccupancyGrid


def transform_to_array(t: Transform) -> np.ndarray:
    e = euler.quat2euler([t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z])
    return np.concatenate([np.array([t.translation.x, t.translation.y]), e])


class Agent(Node):
    def __init__(self):
        qos_profile = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                                    durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                                    history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                    depth=5)
        rclpy.wait_for_message.wait_for_message(OccupancyGrid, Node('waiter'), '/costmap', qos_profile=qos_profile, time_to_wait=-1)
        super().__init__('agent')
        self.get_logger().info('Agent.init')

        self.declare_parameter('agent_namespace', 'ego_racecar')
        self.declare_parameter('opponent_namespace', 'opp_racecar')
        self.declare_parameter('state_topic', 'state')
        self.declare_parameter('drive_topic', 'drive')
        self.declare_parameter('predictions_topic', 'predictions')
        self.declare_parameter('velocity_gain', 1.0)
        self.declare_parameter('opponent_present', False)

        self.declare_parameter('map_name', '')
        self.declare_parameter('map_folder_path', '')
        self.declare_parameter('planner_name', 'sampling')

        self.planner_name = self.get_parameter('planner_name').value
        self.opponent_present: bool = self.get_parameter('opponent_present').value

        self.ego_state: list[float] = None
        agent_namespace: str = self.get_parameter('agent_namespace').value
        drive_topic: str = self.get_parameter('drive_topic').value
        state_topic: str = self.get_parameter('state_topic').value

        self.drive_publiser: rclpy.publisher.Publisher = self.create_publisher(AckermannDriveStamped, f'{agent_namespace}/{drive_topic}', 1)

        self.ego_state_subscriber: rclpy.subscription.Subscription = self.create_subscription(Float64MultiArray, f'{agent_namespace}/{state_topic}', self.ego_state_cb, 10)
            
        if self.opponent_present:
            self.opp_state: list[float] = None
            opponent_namespace: str = self.get_parameter('opponent_namespace').value
            self.opp_state_subscriber: rclpy.subscription.Subscription = self.create_subscription(Float64MultiArray, f'{opponent_namespace}/{state_topic}', self.opp_state_cb, 10)
        else:
            self.opp_state: list[float] = [0,0,0,0,0,0,0]
        
        self.timer: rclpy.timer.Timer = self.create_timer(0.03, self.update, callback_group=rclpy.callback_groups.ReentrantCallbackGroup())


    def plan(self, state: list[float]) -> list[float]:
        raise NotImplementedError()


    def ego_state_cb(self, msg: Float64MultiArray):
        self.ego_state = msg.data


    def opp_state_cb(self, msg: Float64MultiArray):
        self.opp_state = msg.data


    def update(self):
        if self.ego_state and (self.opp_state or not self.opponent_present):
            start = self.get_clock().now()

            action = self.plan(self.ego_state)

            msg = AckermannDriveStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.drive.speed = float(action[1])
            msg.drive.steering_angle = float(action[0])
            self.drive_publiser.publish(msg)

            self.get_logger().info(f"(update) state: x={self.ego_state[0]:.2f}, y={self.ego_state[1]:.2f}, theta={self.ego_state[4]:.2f}, v={self.ego_state[3]:.2f}, d={self.ego_state[2]:.2f}, action: v={action[1]:.2f}, d={action[0]:.2f}, took {(self.get_clock().now() - start).nanoseconds / 1e6} ms")
