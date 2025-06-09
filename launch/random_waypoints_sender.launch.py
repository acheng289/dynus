#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments for configuration
        DeclareLaunchArgument(
            'list_agents',
            default_value="['NX01', 'NX02']", # Example: Default to two agents
            description='List of agent names (e.g., "[\'NX01\', \'NX02\']")'
        ),
        DeclareLaunchArgument(
            'num_waypoints',
            default_value='5',
            description='Number of random waypoints to generate for each agent'
        ),
        DeclareLaunchArgument(
            'x_min',
            default_value='0.0',
            description='Minimum X coordinate for random waypoints'
        ),
        DeclareLaunchArgument(
            'x_max',
            default_value='100.0',
            description='Maximum X coordinate for random waypoints'
        ),
        DeclareLaunchArgument(
            'y_min',
            default_value='-50.0',
            description='Minimum Y coordinate for random waypoints'
        ),
        DeclareLaunchArgument(
            'y_max',
            default_value='50.0',
            description='Maximum Y coordinate for random waypoints'
        ),
        DeclareLaunchArgument(
            'z_min',
            default_value='1.0',
            description='Minimum Z coordinate for random waypoints'
        ),
        DeclareLaunchArgument(
            'z_max',
            default_value='10.0',
            description='Maximum Z coordinate for random waypoints'
        ),
        DeclareLaunchArgument(
            'goal_tolerance',
            default_value='1e-6',
            description='Distance tolerance to consider a goal reached'
        ),
        DeclareLaunchArgument(
            'distance_check_frequency',
            default_value='1.0',
            description='Frequency (Hz) to check the distance to the goal'
        ),
        DeclareLaunchArgument(
            'publish_frequency',
            default_value='1.0',
            description='Frequency (Hz) to publish the current term goal'
        ),
        DeclareLaunchArgument(
            'stuck_time_threshold',
            default_value='5.0',
            description='Time (seconds) before considering the agent stuck'
        ),

        # Launch the RandomWaypointsSender node
        Node(
            package='dynus', # Assuming 'dynus' is the package name where this node resides
            executable='random_waypoints_sender.py',
            name='random_waypoints_sender',
            output='screen',
            parameters=[{
                'list_agents': LaunchConfiguration('list_agents'),
                'num_waypoints': LaunchConfiguration('num_waypoints'),
                'x_min': LaunchConfiguration('x_min'),
                'x_max': LaunchConfiguration('x_max'),
                'y_min': LaunchConfiguration('y_min'),
                'y_max': LaunchConfiguration('y_max'),
                'z_min': LaunchConfiguration('z_min'),
                'z_max': LaunchConfiguration('z_max'),
                'goal_tolerance': LaunchConfiguration('goal_tolerance'),
                'distance_check_frequency': LaunchConfiguration('distance_check_frequency'),
                'publish_frequency': LaunchConfiguration('publish_frequency'),
                'stuck_time_threshold': LaunchConfiguration('stuck_time_threshold'),
            }]
        ),
    ])