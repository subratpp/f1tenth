import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Configuration file path
    # NOTE: Update '/home/f1tenth/...' to your ACTUAL user path
    config_dir = '/home/race_stack/f1tenth/src/cartographer/config'
    config_file = 'f1tenth_2d.lua'

    return LaunchDescription([
        # Cartographer Node
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': False}], # Set True if using simulation
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', config_file
            ],
            remappings=[
                # Ensure your lidar topic matches. F1Tenth often uses /scan
                ('scan', '/scan'),
                ('odom', '/odom'), # Ensure this matches your VESC odom topic
            ]
        ),
        # Occupancy Grid Node (Converts submaps to /map topic)
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': False}, 
                        {'resolution': 0.05}],
        ),
    ])