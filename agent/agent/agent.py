import rclpy
import rclpy.logging
import rclpy.publisher
import rclpy.subscription
import rclpy.time
import rclpy.timer
from rclpy.node import Node
from geometry_msgs.msg import Transform
from ackermann_msgs.msg import AckermannDriveStamped
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
from transforms3d import euler
import numpy as np
from samplingplanner import SamplingPlanner
from state import State
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
        self.declare_parameter('predictions_topic', 'predictions')
        self.declare_parameter('velocity_gain', 1.0)

        self.declare_parameter('map_name', '')
        self.declare_parameter('map_folder_path', '')

        self.ego_state: list[float] = None
        self.opp_state: list[float] = None

        agent_namespace: str = self.get_parameter('agent_namespace').value
        opponent_namespace: str = self.get_parameter('opponent_namespace').value
        drive_topic: str = self.get_parameter('drive_topic').value
        state_topic: str = self.get_parameter('state_topic').value
        predictions_topic: str = self.get_parameter('predictions_topic').value

        self.drive_publiser: rclpy.publisher.Publisher = self.create_publisher(AckermannDriveStamped, f'{agent_namespace}/{drive_topic}', 1)
        self.predictions_publisher: rclpy.publisher.Publisher = self.create_publisher(sensor_msgs.PointCloud2, f'{agent_namespace}/{predictions_topic}', 1)

        self.ego_state_subscriber: rclpy.subscription.Subscription = self.create_subscription(Float64MultiArray, f'{agent_namespace}/{state_topic}', self.ego_state_cb, 10)
        self.opp_state_subscriber: rclpy.subscription.Subscription = self.create_subscription(Float64MultiArray, f'{opponent_namespace}/{state_topic}', self.opp_state_cb, 10)

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

        velocity_gain: float = self.get_parameter('velocity_gain').value
        self.planner: SamplingPlanner = SamplingPlanner(params, Raceline.from_centerline_file(centerline_file), Raceline.from_raceline_file(raceline_file), velocity_gain)

        self.timer: rclpy.timer.Timer = self.create_timer(0.05, self.update)


    def ego_state_cb(self, msg: Float64MultiArray):
        self.ego_state = msg.data

    def opp_state_cb(self, msg: Float64MultiArray):
        self.opp_state = msg.data

    def publish_predictions(self, predictions: list[State]):
        points = np.array([p.position for p in predictions])
        points = np.append(points, np.ones((len(predictions), 1)), axis=1)
        ros_dtype = sensor_msgs.PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize 
        data = points.astype(dtype).tobytes() 

        # The fields specify what the bytes represents. The first 4 bytes 
        # represents the x-coordinate, the next 4 the y-coordinate, etc.
        fields = [sensor_msgs.PointField(name=n, offset=i*itemsize, datatype=ros_dtype, count=1) for i, n in enumerate('xyz')]

        # The PointCloud2 message also has a header which specifies which 
        # coordinate frame it is represented in. 
        header = std_msgs.Header()
        header.frame_id = 'map'

        msg = sensor_msgs.PointCloud2(
            header=header,
            height=1, 
            width=points.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=(itemsize * 3), # Every point consists of three float32s.
            row_step=(itemsize * 3 * points.shape[0]),
            data=data
        )
        self.predictions_publisher.publish(msg)

    def update(self):
        if self.ego_state and self.opp_state:
            start = self.get_clock().now()
            action, predictions = self.planner.plan(self.ego_state, self.opp_state)
            self.publish_predictions(predictions)

            msg = AckermannDriveStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.drive.speed = action[1]
            msg.drive.steering_angle = action[0]
            self.drive_publiser.publish(msg)

            self.get_logger().info(f"(update) state: x={self.ego_state[0]:.2f}, y={self.ego_state[1]:.2f}, theta={self.ego_state[5]:.2f}, v={self.ego_state[3]:.2f}, d={self.ego_state[2]:.2f}, action: v={action[1]:.2f}, d={action[0]:.2f}, took {(self.get_clock().now() - start).nanoseconds / 1e6} ms")


def main():
    rclpy.init()
    agent = Agent()
    rclpy.spin(agent)