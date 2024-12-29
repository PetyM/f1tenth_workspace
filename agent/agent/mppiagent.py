import rclpy
import rclpy.publisher
import rclpy.subscription
import rclpy.timer
import rclpy.action


import numpy as np
import pathlib

from transforms3d import euler

from geometry_msgs.msg import PoseStamped, Twist
from ackermann_msgs.msg import AckermannDriveStamped
from nav2_msgs.action import FollowPath

from f1tenth_gym.envs.track import Raceline


from agent.agentbase import AgentBase
from agent.state import first_point_on_trajectory_intersecting_circle

class MPPIAgent(AgentBase):
    def __init__(self):
        super().__init__()

        self.declare_parameter('map_name', '')
        map_name = self.get_parameter('map_name').value

        self.declare_parameter('map_folder_path', '')
        map_folder_path = self.get_parameter('map_folder_path').value

        centerline = Raceline.from_centerline_file(pathlib.Path(f"{map_folder_path}/{map_name}_centerline.csv"))
        self.centerline = np.flip(np.stack([centerline.xs, centerline.ys], dtype=np.float64).T, 0)

        self.waypoint_distance: float = 20.0

        self.waypoint_publisher: rclpy.publisher.Publisher = self.create_publisher(PoseStamped, '/goal_pose', 1)
        self.cmd_subscription: rclpy.subscription.Subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 1)

        self.action: rclpy.action.ActionClient = rclpy.action.ActionClient(self, FollowPath, '/follow_path')

        self.timer_update_control: rclpy.timer.Timer = self.create_timer(0.3, self.update_control)


    def cmd_callback(self, cmd: Twist):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = cmd.linear.x
        msg.drive.steering_angle = cmd.angular.z
        self.drive_publiser.publish(msg)


    def update_control(self):
        start = self.get_clock().now()

        waypoint, _, _ = first_point_on_trajectory_intersecting_circle(np.array(self.ego_state[:2], dtype=np.float64), self.waypoint_distance, self.centerline, wrap=True)

        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        message.pose.position.x = float(waypoint[0])
        message.pose.position.y = float(waypoint[1])
        message.pose.position.z = float(0.0)

        quat = euler.euler2quat(0.0, 0.0, 0.0, axes='sxyz')
        message.pose.orientation.x = float(quat[1])
        message.pose.orientation.y = float(quat[2])
        message.pose.orientation.z = float(quat[3])
        message.pose.orientation.w = float(quat[0])
        
        self.waypoint_publisher.publish(message)

        msg = FollowPath.Goal()
        msg.path.poses.append(message)
        self.action.wait_for_server()
        self.action.send_goal_async(msg)

        self.get_logger().warn(f"AgentBase.update: State: took {(self.get_clock().now() - start).nanoseconds / 1e6} ms")


def main():
    rclpy.init()
    agent = MPPIAgent()
    rclpy.spin(agent)