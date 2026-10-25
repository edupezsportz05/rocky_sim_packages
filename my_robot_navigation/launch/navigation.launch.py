import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav = get_package_share_directory('my_robot_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # --- Launch arguments: everything robot/environment-specific lives here ---
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('my_map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true, real robot clock if false'
    )

    declare_map_yaml = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_nav, 'maps', 'my_map.yaml'),
        description='Full path to map yaml file to load'
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_nav, 'config', 'nav2_params.yaml'),
        description='Full path to the Nav2 parameters file'
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically bring the Nav2 lifecycle nodes to ACTIVE'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz alongside navigation'
    )

    launch_filter = LaunchConfiguration('launch_filter')
    declare_launch_filter = DeclareLaunchArgument(
        'launch_filter',
        default_value='false',
        description='Whether to launch scan_filter node (set true if not already running in simulation)'
    )

    scan_filter_node = Node(
        package='my_robot_navigation',
        executable='scan_filter.py',
        name='scan_filter',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(launch_filter)
    )

    # --- Core Nav2 stack (map_server, amcl, planner, controller, bt_navigator, etc.) ---
    # This does NOT spawn a robot or start Gazebo — it only assumes /scan, /odom
    # and the TF tree are already being published by whatever robot is running.
    
    nav2_bringup_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(
            pkg_nav2_bringup,
            "launch",
            "bringup_launch.py"
        )
    )
)



    # --- Optional RViz, using a config file that only references generic frames/topics ---
    rviz_config = os.path.join(pkg_nav, 'rviz', 'navigation.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map_yaml,
        declare_params_file,
        declare_autostart,
        declare_use_rviz,
        declare_launch_filter,
        scan_filter_node,
        nav2_bringup_launch,
        rviz_node,
    ])
