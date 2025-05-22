#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseStamped
from dynus_interfaces.msg import Goal, State
from message_filters import Subscriber, ApproximateTimeSynchronizer

class StateGoalCollector(Node):
    def __init__(self):
        super().__init__("state_goal_collector")
        self.state_sub = self.create_subscription(State, "state", self.state_callback, QoSProfile(depth=10))
        self.prev_state = None
    
    def state_callback(self, incoming_state):
        if self.prev_state is None:
            self.prev_state = incoming_state
            return
        
        delta_s = incoming_state.header.stamp.sec - self.prev_state.header.stamp.sec
        delta_ns = incoming_state.header.stamp.nanosec - self.prev_state.header.stamp.nanosec
        delta_t = delta_s + delta_ns * 1e-9

        self.get_logger().info(f"Delta time: {delta_t:.6f} seconds")
        self.prev_state = incoming_state

def main(args=None):
    rclpy.init(args=args)
    node = StateGoalCollector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
