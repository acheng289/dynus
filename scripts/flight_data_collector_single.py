#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from dynus_interfaces.msg import Goal, State
import os
import csv

class StateGoalCollector(Node):
    def __init__(self):
        super().__init__("state_goal_collector")

        # Creating parameters
        self.declare_parameter('agent_name', 'default_agent')
        self.declare_parameter('output_dir', os.path.join(os.getcwd(), 'flight_data_root'))
        self.declare_parameter('scene_name', 'default_scene')

        # Get parameters
        self.agent_name= self.get_parameter('agent_name').value
        self.root_output_dir = self.get_parameter('output_dir').value
        self.scene_name = self.get_parameter('scene_name').value

        # Construct output directory
        self.full_output_dir = os.path.join(self.root_output_dir, self.scene_name)
        os.makedirs(self.full_output_dir, exist_ok=True)

        csv_filename = os.path.join(self.full_output_dir, f'{self.agent_name}.csv')
        self.csv_file = open(csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp_sec', 'timestamp_nanosec',
            'delta_t_sec', 'pos_x', 'pos_y',
            'pos_z', 'vel_x', 'vel_y', 'vel_z',
            'q_x', 'q_y', 'q_z', 'q_w'
        ])

        self.state_sub = self.create_subscription(State, "state", self.state_callback, QoSProfile(depth=10))
        self.prev_state = None
        self.get_logger().info(f'Collecting flight information for {self.agent_name} in {csv_filename}')
    
    def state_callback(self, incoming_state):
        delta_t = 0.0
        if self.prev_state is not None:
            delta_s = incoming_state.header.stamp.sec - self.prev_state.header.stamp.sec
            delta_ns = incoming_state.header.stamp.nanosec - self.prev_state.header.stamp.nanosec
            delta_t = delta_s + delta_ns * 1e-9

        # Write data to CSV
        self.csv_writer.writerow([
            incoming_state.header.stamp.sec,
            incoming_state.header.stamp.nanosec,
            f"{delta_t:.6f}",
            f"{incoming_state.pos.x:.6f}",
            f"{incoming_state.pos.y:.6f}",
            f"{incoming_state.pos.z:.6f}",
            f"{incoming_state.quat.x:.6f}",
            f"{incoming_state.quat.y:.6f}",
            f"{incoming_state.quat.z:.6f}",
            f"{incoming_state.quat.w:.6f}"
        ])

        self.prev_state = incoming_state

    def destroy_node(self):
        self.get_logger().info(f"Closing CSV file for agent {self.agent_namespace}.")
        if self.csv_file:
            self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = StateGoalCollector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
