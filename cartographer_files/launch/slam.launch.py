#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_gaz_sim = get_package_share_directory('gaz_sim')
    
    slam_params = os.path.join(pkg_gaz_sim, 'config', 'slam_toolbox_params.yaml')
    
    # SLAM Toolbox - Online Sync (live mapping)
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='online_async_launch',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params,
            {'use_sim_time': True}
        ],
        remappings=[
            ('scan', '/scan'),
        ]
    )
    
    # Map saver (save map when done)
    map_saver_cli = Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        name='map_saver_cli',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-f', 'my_map']
    )
    
    # RViz with SLAM config
    rviz_config = os.path.join(pkg_gaz_sim, 'rviz', 'slam.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    ld = LaunchDescription()
    ld.add_action(slam_toolbox)
    ld.add_action(map_saver_cli)
    ld.add_action(rviz)
    
    return ld

