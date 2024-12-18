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

from std_msgs.msg import Float64MultiArray


class AgentBase(rclpy.node.Node):
    def __init__(self):
        super().__init__('agent')

        self.declare_parameter('agent_namespace', 'ego_racecar')
        self.agent_namespace: str = self.get_parameter('agent_namespace').value

        self.declare_parameter('opponent_namespace', 'opp_racecar')
        self.opponent_namespace: str = self.get_parameter('opponent_namespace').value

        self.declare_parameter('state_topic', 'state')
        self.state_topic: str = self.get_parameter('state_topic').value

        self.declare_parameter('drive_topic', 'drive')
        self.drive_topic: str = self.get_parameter('drive_topic').value

        self.declare_parameter('velocity_gain', 1.0)
        self.velocity_gain: float =  self.get_parameter('velocity_gain').value

        self.declare_parameter('opponent_present', False)
        self.opponent_present: bool = self.get_parameter('opponent_present').value

        self.drive_publiser: rclpy.publisher.Publisher = self.create_publisher(AckermannDriveStamped, f'{self.agent_namespace}/{self.drive_topic}', 1)

        self.ego_state_subscriber: rclpy.subscription.Subscription = self.create_subscription(Float64MultiArray, f'{self.agent_namespace}/{self.state_topic}', self.ego_state_cb, 10)
        if self.opponent_present:
            self.opp_state_subscriber: rclpy.subscription.Subscription = self.create_subscription(Float64MultiArray, f'{self.opponent_namespace}/{self.state_topic}', self.opp_state_cb, 10)
        
        self.timer_update_control: rclpy.timer.Timer = self.create_timer(0.03, self.update_control)

        self.ego_state: list[float] = [0,0,0,0,0,0,0]
        self.opp_state: list[float] = [0,0,0,0,0,0,0]


    def plan(self, state: list[float]) -> list[float]:
        raise NotImplementedError()


    def ego_state_cb(self, msg: Float64MultiArray):
        self.ego_state = msg.data


    def opp_state_cb(self, msg: Float64MultiArray):
        self.opp_state = msg.data


    def update_control(self):
        start = self.get_clock().now()

        action = self.plan(self.ego_state)

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(action[1])
        msg.drive.steering_angle = float(action[0])
        self.drive_publiser.publish(msg)

        self.get_logger().info(f"AgentBase.update: State: v={self.ego_state[3]:.2f}, d={self.ego_state[2]:.2f}, action: v={action[1]:.2f}, d={action[0]:.2f}, took {(self.get_clock().now() - start).nanoseconds / 1e6} ms")
