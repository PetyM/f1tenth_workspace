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
import cv2

from ackermann_msgs.msg import AckermannDriveStamped

from std_msgs.msg import Float64MultiArray, Bool
from std_srvs.srv import Empty
import math

class Detector(rclpy.node.Node):
    def __init__(self, node_name: str= 'detector'):
        super().__init__(node_name)

        self.declare_parameter('agent_namespace', 'ego_racecar')
        self.agent_namespace: str = self.get_parameter('agent_namespace').value

        self.declare_parameter('opponent_namespace', 'opp_racecar')
        self.opponent_namespace: str = self.get_parameter('opponent_namespace').value

        self.declare_parameter('state_topic', 'state')
        self.state_topic: str = self.get_parameter('state_topic').value

        self.ego_state_subscriber: rclpy.subscription.Subscription = self.create_subscription(Float64MultiArray, f'{self.agent_namespace}/{self.state_topic}', self.ego_state_cb, 10)
        self.opp_state_subscriber: rclpy.subscription.Subscription = self.create_subscription(Float64MultiArray, f'{self.opponent_namespace}/{self.state_topic}', self.opp_state_cb, 10)

        self.timer: rclpy.timer.Timer = self.create_timer(0.01, self.timer_cb)
        self.vehicle_width = 0.31
        self.vehicle_length = 0.58
        self.get_logger().warn('Detector ready')

        self.ego_state: list[float] = [10,0,0,0,0,0,0]
        self.opp_state: list[float] = [0,0,0,0,0,0,0]


    def ego_state_cb(self, msg: Float64MultiArray):
        self.ego_state = msg.data


    def opp_state_cb(self, msg: Float64MultiArray):
        self.opp_state = msg.data

    def timer_cb(self):
        collision, _ = cv2.rotatedRectangleIntersection(((self.ego_state[0], self.ego_state[1]), (self.vehicle_width, self.vehicle_length), math.degrees(self.ego_state[2])),
                                                        ((self.opp_state[0], self.opp_state[1]), (self.vehicle_width, self.vehicle_length), math.degrees(self.opp_state[2])))
        if collision != 0:
            self.get_logger().warn("Vehicle collision")


def main():
    rclpy.init()
    agent = Detector()
    rclpy.spin(agent)
