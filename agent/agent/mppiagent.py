import rclpy

from agent.agentbase import AgentBase

class MPPIAgent(AgentBase):
    def __init__(self):
        super().__init__()

    def plan(self, state):
        return [0, 0]


def main():
    rclpy.init()
    agent = MPPIAgent()
    rclpy.spin(agent)