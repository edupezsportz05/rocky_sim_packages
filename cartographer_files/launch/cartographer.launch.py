#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_cartographer_config = get_package_share_directory('cartographer_config')
    
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz2'
    )
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Static TF bridge for Gazebo sensor frame compatibility
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gazebo_lidar_tf_bridge',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'lidar_1',
            '--child-frame-id', 'my_amr/base_link/gpu_lidar'
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Cartographer Node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', os.path.join(pkg_cartographer_config, 'config'),
            '-configuration_basename', 'my_robot.lua'
        ],
        remappings=[
            ('scan', '/scan'),           # Map to your LIDAR topic
            ('imu', '/imu'),             # Map to your IMU topic
            ('odom', '/odom'),           # Map to your odometry topic
        ]
    )
    
    # Cartographer Occupancy Grid Node
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', '0.05']  # 5cm grid resolution
    )
    
    # RViz2 Node (optional)
    rviz_config_dir = os.path.join(pkg_cartographer_config, 'rviz', 'cartographer.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config_dir],
        condition=IfCondition(use_rviz)
    )
    
    return LaunchDescription([
        declare_use_rviz_cmd,
        declare_use_sim_time_cmd,
        static_tf_node,
        cartographer_node,
        occupancy_grid_node,
        rviz_node
    ])
