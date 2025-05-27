#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction # Import OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

# Define a function to generate the nodes based on resolved arguments
def generate_nodes_from_agents(context, *args, **kwargs):
    list_agents_str = LaunchConfiguration('list_agents').perform(context)
    # Safely parse the string representation of the list
    # Use ast.literal_eval for robust parsing of Python literals
    import ast
    try:
        list_agents = ast.literal_eval(list_agents_str)
        if not isinstance(list_agents, list):
            raise ValueError("list_agents must be a list string.")
    except (ValueError, SyntaxError) as e:
        print(f"Error parsing list_agents: {list_agents_str}. Please ensure it's a valid Python list string. Error: {e}")
        return [] # Return empty list to prevent crash


    output_dir = LaunchConfiguration('output_dir')
    scene_name = LaunchConfiguration('scene_name')

    nodes_to_launch = []
    for agent in list_agents:
        nodes_to_launch.append(
            Node(
                package='dynus',
                executable='flight_data_collector.py',
                name=['flight_data_collector_', agent], # Unique name for each node
                namespace=agent, # Set the namespace for the node
                output='screen',
                parameters=[{
                    'agent_name': agent, # Pass the agent's namespace to the node for CSV naming
                    'output_dir': output_dir, # Use LaunchConfiguration directly here
                    'scene_name': scene_name, # Use LaunchConfiguration directly here
                }],
            )
        )
    return nodes_to_launch


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

        # Use OpaqueFunction to defer node creation until arguments are resolved
        OpaqueFunction(function=generate_nodes_from_agents),
    ])