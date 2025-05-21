#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseStamped
from dynus_interfaces.msg import Goal, State
from message_filters import Subscriber, TimeSynchronizer

class StateGoalCollector(Node):
    def __init__(self):
        super().__init__("state_goal_collector")
        self.state_sub = Subscriber(self, State, "state")
        self.goal_sub = Subscriber(self, Goal, "goal")
        qos = QoSProfile(depth=10)

        queue_size = 10
        self.sync = TimeSynchronizer([self.state_sub, self.goal_sub], queue_size)
        self.sync.registerCallback(self.sync_callback)
    
    def sync_callback(self, state_msg, goal_msg):
        state_sec = state_msg.header.stamp.sec
        goal_sec = goal_msg.header.stamp.sec
        self.get_logger().info(f"sync callback wit {state_sec} and {goal_sec}")

def main(args=None):
    rclpy.init(args=args)
    node = StateGoalCollector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
