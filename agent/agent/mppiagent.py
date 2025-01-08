import rclpy
import rclpy.publisher
import rclpy.subscription
import rclpy.task
import rclpy.timer
import rclpy.action


import numpy as np
import pathlib

from transforms3d import euler

from geometry_msgs.msg import PoseStamped, Twist
from ackermann_msgs.msg import AckermannDriveStamped
from nav2_msgs.action import FollowPath
from nav2_msgs.msg import SpeedLimit

from f1tenth_gym.envs.track import Raceline


from agent.agentbase import AgentBase
from agent.state import first_point_on_trajectory_intersecting_circle, nearest_point_on_trajectory

class MPPIAgent(AgentBase):
    def __init__(self):
        super().__init__()

        self.declare_parameter('map_name', '')
        map_name = self.get_parameter('map_name').value

        self.declare_parameter('map_folder_path', '')
        map_folder_path = self.get_parameter('map_folder_path').value

        centerline = Raceline.from_centerline_file(pathlib.Path(f"{map_folder_path}/{map_name}_centerline.csv"))
        self.centerline = np.flip(np.stack([centerline.xs, centerline.ys], dtype=np.float64).T, 0)

        self.waypoint_distance: float = 10.0

        self.waypoint_publisher: rclpy.publisher.Publisher = self.create_publisher(PoseStamped, '/goal_pose', 1)
        self.cmd_subscription: rclpy.subscription.Subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 1)

        self.action: rclpy.action.ActionClient = rclpy.action.ActionClient(self, FollowPath, '/follow_path')
        self.future: rclpy.task.Future = None

        self.timer_update_control: rclpy.timer.Timer = self.create_timer(1.0, self.update_control)

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

        self.wheelbase = params["lr"] + params["lf"]

        self.action.wait_for_server()

        speed_limit_publisher = self.create_publisher(SpeedLimit, '/speed_limit', 1)
        limit = SpeedLimit()
        limit.header.stamp = self.get_clock().now().to_msg()
        limit.header.frame_id = 'map'
        limit.percentage = True
        limit.speed_limit = 100.0
        speed_limit_publisher.publish(limit)



    def cmd_callback(self, cmd: Twist):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = cmd.linear.x
        msg.drive.steering_angle =  np.arctan2(cmd.angular.z * self.wheelbase, cmd.linear.x)
        self.drive_publiser.publish(msg)
        self.get_logger().info(f"MPPIAgent.cmd_callback: CMD: {cmd.linear.x}, {cmd.angular.z}")


    def update_control(self):
        if self.future is not None:
            if self.future.done():
                self.get_logger().warn(f"future {self.future.result()}")
                self.future = None
            else:
                return
        start = self.get_clock().now()

        _, _, _, trajectory_index = nearest_point_on_trajectory(np.array(self.ego_state[:2], dtype=np.float64), self.centerline)
        waypoint, waypoint_index, _ = first_point_on_trajectory_intersecting_circle(np.array(self.ego_state[:2], dtype=np.float64), self.waypoint_distance, self.centerline, trajectory_index, True)
        waypoint_n1 = self.centerline[waypoint_index + 1, :]
        
        waypoint_heading_vector = waypoint_n1 - waypoint
        angle_radians = np.arctan2(waypoint_heading_vector[1], waypoint_heading_vector[0])

        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        message.pose.position.x = float(waypoint[0])
        message.pose.position.y = float(waypoint[1])
        message.pose.position.z = float(0.0)

        quat = euler.euler2quat(0.0, 0.0, angle_radians, axes='sxyz')
        message.pose.orientation.x = float(quat[1])
        message.pose.orientation.y = float(quat[2])
        message.pose.orientation.z = float(quat[3])
        message.pose.orientation.w = float(quat[0])
        
        self.waypoint_publisher.publish(message)

        msg = FollowPath.Goal()
        msg.controller_id = 'FollowPath'
        msg.path.header.frame_id = 'map'
        msg.path.header.stamp = message.header.stamp
        msg.path.poses.append(message)
        self.future = self.action.send_goal_async(msg)

        self.get_logger().warn(f"MPPIAgent.update: State: took {(self.get_clock().now() - start).nanoseconds / 1e6} ms")


def main():
    rclpy.init()
    agent = MPPIAgent()
    rclpy.spin(agent)