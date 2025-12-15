import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Dynamic Path Handling
    # Change 'f1tenth_stack' to the actual name of your ROS 2 package where config is stored
    pkg_share = get_package_share_directory('cartographer') 
    
    # Alternatively, if your config is in your own package (recommended):
    # pkg_share = get_package_share_directory('my_f1tenth_package')

    # 2. Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')
    
    # 3. Path to Lua Config
    # Assuming the file is inside the 'configuration_files' or 'config' folder of the package
    config_dir = os.path.join(pkg_share, 'config')
    config_file = 'f1tenth_2d.lua'

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation (Gazebo) clock if true'),

        # Cartographer Node
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', config_file
            ],
            remappings=[
                ('scan', '/scan'),
                ('odom', '/odom'),
                # ('imu', '/imu/data'), # Uncomment if using IMU in Lua
            ]
        ),

        # Occupancy Grid Node
        Node(
            package='cartographer_ros',
            executable='occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'resolution': 0.05}
            ],
        ),
    ])