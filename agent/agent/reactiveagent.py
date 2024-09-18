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
from sensor_msgs.msg import LaserScan

from std_msgs.msg import Float64MultiArray, Bool
from std_srvs.srv import Empty
from f1tenth_gym.envs.dynamic_models import pid_steer, pid_accl

import numpy as np
import agent.drivers

class ReactiveAgent(rclpy.node.Node):
    def __init__(self):
        super().__init__('agent')

        self.declare_parameter('agent_namespace', 'ego_racecar')
        self.agent_namespace: str = self.get_parameter('agent_namespace').value

        self.declare_parameter('driver', 'PureFTG')
        DRIVERS = {
            'FTG': agent.drivers.GapFollower,
            'DE': agent.drivers.DisparityExtender,
            'FTGmax': agent.drivers.GapFollowerMax,
            'DEmax' : agent.drivers.DisparityExtenderMax,
        }
        self.driver = DRIVERS[self.get_parameter('driver').value]()

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
    
        self.ego_state_subscriber: rclpy.subscription.Subscription = self.create_subscription(Float64MultiArray, f'{self.agent_namespace}/state', self.ego_state_cb, 10)
        self.scan_subcriber: rclpy.subscription.Subscription = self.create_subscription(LaserScan, f'{self.agent_namespace}/scan', self.scan_cb, 1)
        self.drive_publiser: rclpy.publisher.Publisher = self.create_publisher(AckermannDriveStamped, f'{self.agent_namespace}/drive', 1)
        self.timer_update_control: rclpy.timer.Timer = self.create_timer(0.001, self.update_control)
        self.target_speed: float = 0.0
        self.target_steering_angle: float = 0.0
        self.ego_state: list[float] = [0,0,0,0,0,0,0]

        self.ready()

    def ego_state_cb(self, msg: Float64MultiArray):
        self.ego_state = msg.data

    def ready(self):
        client = self.create_client(Empty, f'{self.agent_namespace}/ready')
        client.wait_for_service()
        client.call_async(Empty.Request())
        self.get_logger().info('Ready')

    
    def scan_cb(self, scan: LaserScan):
        speed, steering_angle = self.driver.process_lidar(scan)

        self.target_speed = speed
        self.target_steering_angle = steering_angle
        self.get_logger().info(f'{speed=}, {steering_angle=} ')

    def update_control(self):
        acceleration = pid_accl(self.target_speed, self.ego_state[3], self.params["a_max"], self.params["v_max"] * self.velocity_gain, self.params["v_min"],)
        steering = pid_steer(self.target_steering_angle, self.ego_state[2], self.params["sv_max"])
        self.get_logger().info(f'{acceleration=}, {steering=} ')

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.acceleration = float(acceleration)
        msg.drive.steering_angle_velocity = float(steering)
        self.drive_publiser.publish(msg)

def main():
    rclpy.init()
    agent = ReactiveAgent()
    rclpy.spin(agent)