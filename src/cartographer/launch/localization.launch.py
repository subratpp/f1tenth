import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # ---------------------------------------------------------
    # 1. SETUP & ARGUMENTS
    # ---------------------------------------------------------
    # Change 'stack_master' to your actual package name if different
    package_name = 'stack_master'
    pkg_share = get_package_share_directory(package_name)

    # Launch Configurations
    mapping_bool = LaunchConfiguration('mapping_bool')
    racecar_version = LaunchConfiguration('racecar_version')
    map_name = LaunchConfiguration('map_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    frenet_bool = LaunchConfiguration('frenet_bool')

    # Construct Dynamic Paths
    # Path to config: .../stack_master/config/NUC2
    config_dir = PathJoinSubstitution([pkg_share, 'config', racecar_version])
    
    # Path to pbstream: .../stack_master/maps/hangar/hangar.pbstream
    pbstream_file = PathJoinSubstitution([
        pkg_share, 'maps', map_name, 
        PythonExpression(["'", map_name, ".pbstream'"])
    ])

    # Path to yaml: .../stack_master/maps/hangar/hangar.yaml
    yaml_file = PathJoinSubstitution([
        pkg_share, 'maps', map_name, 
        PythonExpression(["'", map_name, ".yaml'"])
    ])

    # ---------------------------------------------------------
    # 2. NODES
    # ---------------------------------------------------------
    
    # [LOCALIZATION] Cartographer Node
    # Runs when mapping_bool is FALSE. Loads .pbstream state.
    cartographer_loc_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', 'f110_2d_loc.lua',
            '-load_state_filename', pbstream_file,
            '--minloglevel', '2'
        ],
        remappings=[
            ('odom', '/odom'),  # Direct VESC odometry (No EKF)
            ('scan', '/scan'),
        ],
        condition=UnlessCondition(mapping_bool)
    )

    # [LOCALIZATION] Map Server (Nav2)
    # Publishes the static map for visualization/planning
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': yaml_file,
            'use_sim_time': use_sim_time
        }],
        condition=UnlessCondition(mapping_bool)
    )

    # [LOCALIZATION] Lifecycle Manager
    # Required to bring the map_server to 'Active' state
    lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server']
        }],
        condition=UnlessCondition(mapping_bool)
    )

    # [MAPPING] Cartographer Node
    # Runs when mapping_bool is TRUE. Starts fresh map.
    cartographer_map_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', 'f110_2d.lua',
            '--minloglevel', '2'
        ],
        remappings=[
            ('odom', '/odom'),
            ('scan', '/scan'),
        ],
        condition=IfCondition(mapping_bool)
    )

    # [SHARED] Occupancy Grid Node
    # Converts internal submaps to /map topic
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node', # Check if your install uses 'occupancy_grid_node' instead
        name='cartographer_occupancy_grid_node',
        output='screen',
        arguments=['-resolution', '0.05'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # [SHARED] CarState Node
    carstate_node = Node(
        package='state_estimation',
        executable='carstate_node',
        name='carstate_node',
        output='screen',
        parameters=[{
            'frenet_bool': frenet_bool,
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('/odom_out', '/car_state/odom')
        ]
    )

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('mapping_bool', default_value='False', description='False = Localization, True = Mapping'),
        DeclareLaunchArgument('racecar_version', default_value='NUC2'),
        DeclareLaunchArgument('map_name', default_value='hangar'),
        DeclareLaunchArgument('frenet_bool', default_value='False'),
        DeclareLaunchArgument('use_sim_time', default_value='False'),

        # Nodes
        cartographer_loc_node,
        map_server_node,
        lifecycle_node,
        cartographer_map_node,
        occupancy_grid_node,
        carstate_node
    ])