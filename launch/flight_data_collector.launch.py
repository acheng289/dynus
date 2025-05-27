#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():

    # Parameters (example, adjust as needed)
    list_agents_default = ['NX01', 'NX02'] # Default agents to collect data for
    default_scene_name = 'scene_01' # Default scene name

    return LaunchDescription([
        # Declare the launch arguments
        DeclareLaunchArgument(
            'list_agents',
            default_value=str(list_agents_default), # Ensure it's a string representation of the list
            description='List of agent names to collect data for (e.g., "[\'NX01\', \'NX02\']")'
        ),
        DeclareLaunchArgument(
            'output_dir',
            default_value=PathJoinSubstitution([os.getcwd(), 'scenes']), # Base directory for all scenes
            description='Root directory where all scene data will be saved'
        ),
        DeclareLaunchArgument(
            'scene_name',
            default_value=default_scene_name,
            description='Name of the current scene (e.g., "scene_01")'
        ),

        # Create a list of Node actions based on list_agents
        # This will be processed at launch time
        Node(
            package='dynus', # Or your package name where flight_data_collector.py resides
            executable='flight_data_collector.py',
            name=['flight_data_collector_', agent], # Unique name for each node
            namespace=agent, # Set the namespace for the node
            output='screen',
            parameters=[{
                'agent_name': agent, # Pass the agent's namespace to the node for CSV naming
                'output_dir': LaunchConfiguration('output_dir'),
                'scene_name': LaunchConfiguration('scene_name'), # Pass the scene name
            }],
        ) for agent in LaunchConfiguration('list_agents').perform(None).strip("[]").replace("'", "").split(', ') # Dynamically parse the list of agents
    ])

# ros2 launch dynus flight_data_collector.launch.py list_agents="['NX01', 'NX02']" scene_name:='scene_02'
