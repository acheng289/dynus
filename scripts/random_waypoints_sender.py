#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import Header
from dynus_interfaces.msg import State 
import math
import random

class RandomWaypointsSender(Node):
    def __init__(self):
        super().__init__('random_waypoints_sender')

        # Declare parameters
        self.declare_parameter('list_agents', ['NX01'])
        self.declare_parameter('num_waypoints', 5)
        self.declare_parameter('x_min', 0.0)
        self.declare_parameter('x_max', 100.0)
        self.declare_parameter('y_min', -50.0)
        self.declare_parameter('y_max', 50.0)
        self.declare_parameter('z_min', 1.0)
        self.declare_parameter('z_max', 10.0)
        self.declare_parameter('goal_tolerance', 0.5)
        self.declare_parameter('distance_check_frequency', 1.0)
        self.declare_parameter('publish_frequency', 1.0)
        self.declare_parameter('stuck_time_threshold', 5.0)

        # Get parameters
        self.list_agents = self.get_parameter('list_agents').value
        self.num_waypoints = self.get_parameter('num_waypoints').value
        self.x_min = self.get_parameter('x_min').value
        self.x_max = self.get_parameter('x_max').value
        self.y_min = self.get_parameter('y_min').value
        self.y_max = self.get_parameter('y_max').value
        self.z_min = self.get_parameter('z_min').value
        self.z_max = self.get_parameter('z_max').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.distance_check_frequency = self.get_parameter('distance_check_frequency').value
        self.publish_frequency = self.get_parameter('publish_frequency').value
        self.stuck_time_threshold = self.get_parameter('stuck_time_threshold').value

        self.get_logger().info(f"Agents: {self.list_agents}")
        self.get_logger().info(f"Number of waypoints per agent: {self.num_waypoints}")
        self.get_logger().info(f"X bounds: [{self.x_min}, {self.x_max}]")
        self.get_logger().info(f"Y bounds: [{self.y_min}, {self.y_max}]")
        self.get_logger().info(f"Z bounds: [{self.z_min}, {self.z_max}]")
        self.get_logger().info(f"Goal tolerance: {self.goal_tolerance}")

        # Store agent-specific data
        self.agent_data = {}
        for agent in self.list_agents:
            self.agent_data[agent] = {
                'waypoints': self.generate_random_waypoints(),
                'current_waypoint_index': 0,
                'current_position': Vector3(),
                'state_sub': self.create_subscription(State, f'/{agent}/state',
                                                      lambda msg, a=agent: self.state_callback(msg, a), 10),
                'term_goal_pub': self.create_publisher(PoseStamped, f'/{agent}/term_goal', 10),
                'goal_timer': self.create_timer(self.distance_check_frequency,
                                                lambda a=agent: self.distance_check_callback(a)),
                'publish_timer': self.create_timer(self.publish_frequency,
                                                   lambda a=agent: self.publish_term_goal(a))
            }
            self.get_logger().info(f"Generated waypoints for {agent}: {self.agent_data[agent]['waypoints']}")

    def generate_random_waypoints(self):
        waypoints = []
        for _ in range(self.num_waypoints):
            x = random.uniform(self.x_min, self.x_max)
            y = random.uniform(self.y_min, self.y_max)
            z = random.uniform(self.z_min, self.z_max)
            waypoints.append([x, y, z])
        return waypoints

    def state_callback(self, msg: State, agent_name: str):
        """Callback for monitoring the drone's position for a specific agent."""
        self.agent_data[agent_name]['current_position'] = msg.pos

    def distance_check_callback(self, agent_name: str):
        agent_info = self.agent_data[agent_name]
        current_waypoint_index = agent_info['current_waypoint_index']
        waypoints = agent_info['waypoints']

        if not waypoints:
            self.get_logger().warn(f"No waypoints defined for {agent_name}. Skipping distance check.")
            return

        # Get the current goal point
        goal_x, goal_y, goal_z = waypoints[current_waypoint_index]

        # Compute the Euclidean distance to the current goal
        current_pos = agent_info['current_position']
        distance = math.sqrt(
            (current_pos.x - goal_x) ** 2 +
            (current_pos.y - goal_y) ** 2 +
            (current_pos.z - goal_z) ** 2
        )

        # Initialize stuck parameters if they don't exist
        if 'previous_distance' not in agent_info:
            agent_info['previous_distance'] = distance
            agent_info['stuck_start_time'] = self.get_clock().now()
            return  # Skip the first check

        # Check if the drone has been stuck
        if abs(distance - agent_info['previous_distance']) < 0.1:  # Define a small threshold for "stuck"
            if 'stuck_start_time' not in agent_info:
                agent_info['stuck_start_time'] = self.get_clock().now()
            else:
                time_since_stuck = (self.get_clock().now() - agent_info['stuck_start_time']).to_msg().nanosec / 1e9
                if time_since_stuck > self.stuck_time_threshold:
                    self.get_logger().warn(f"[{agent_name}] Agent is stuck! Moving to next waypoint.")
                    # Move to the next waypoint if not the last one
                    if current_waypoint_index < len(waypoints) - 1:
                        agent_info['current_waypoint_index'] += 1
                        self.get_logger().info(f"[{agent_name}] Moving to next waypoint: {agent_info['waypoints'][agent_info['current_waypoint_index']]}")
                    else:
                        self.get_logger().info(f"[{agent_name}] All waypoints completed. Staying at the last waypoint.")
                    # Reset stuck parameters
                    agent_info['stuck_start_time'] = self.get_clock().now()
                    agent_info['previous_distance'] = distance  # Update previous distance
                    return
        else:
            # Reset stuck parameters if the drone is moving
            agent_info['stuck_start_time'] = self.get_clock().now()

        agent_info['previous_distance'] = distance
        self.get_logger().info(f"[{agent_name}] Distance to goal {current_waypoint_index}: {distance:.2f}")

        # Check if the drone has reached the current goal
        if distance < self.goal_tolerance:
            self.get_logger().info(f"[{agent_name}] Goal {current_waypoint_index} reached!")
            # Move to the next waypoint if not the last one
            if current_waypoint_index < len(waypoints) - 1:
                agent_info['current_waypoint_index'] += 1
                self.get_logger().info(f"[{agent_name}] Moving to next waypoint: {agent_info['waypoints'][agent_info['current_waypoint_index']]}")
            else:
                self.get_logger().info(f"[{agent_name}] All waypoints completed. Staying at the last waypoint.")
            agent_info['stuck_start_time'] = self.get_clock().now()
            agent_info['previous_distance'] = distance

    def publish_term_goal(self, agent_name: str):
        """Publishes the current goal as a PoseStamped message for a specific agent."""
        agent_info = self.agent_data[agent_name]
        current_waypoint_index = agent_info['current_waypoint_index']
        waypoints = agent_info['waypoints']

        if not waypoints:
            self.get_logger().warn(f"No waypoints defined for {agent_name}. Skipping goal publication.")
            return

        goal_x, goal_y, goal_z = waypoints[current_waypoint_index]

        # Create PoseStamped message
        term_goal = PoseStamped()
        term_goal.header = Header()
        term_goal.header.stamp = self.get_clock().now().to_msg()
        term_goal.header.frame_id = "map" 

        term_goal.pose.position.x = goal_x
        term_goal.pose.position.y = goal_y
        term_goal.pose.position.z = goal_z

        term_goal.pose.orientation.x = 0.0
        term_goal.pose.orientation.y = 0.0
        term_goal.pose.orientation.z = 0.0
        term_goal.pose.orientation.w = 1.0  # Identity quaternion

        # Publish the term goal
        agent_info['term_goal_pub'].publish(term_goal)
        self.get_logger().info(f"[{agent_name}] Published term goal {current_waypoint_index}: [{goal_x:.2f}, {goal_y:.2f}, {goal_z:.2f}]")


def main(args=None):
    rclpy.init(args=args)
    node = RandomWaypointsSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()