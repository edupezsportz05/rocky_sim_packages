import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_nav = get_package_share_directory('my_robot_navigation')

    # Include TB3's world/robot spawn, but we'll override the bridge separately
    tb3_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'turtlebot3_world.launch.py')
        )
    )

    # Corrected bridge — ONLY cmd_vel, nothing else duplicated
    fixed_cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge_fix',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
        output='screen'
    )

    return LaunchDescription([
        tb3_world,
        fixed_cmd_vel_bridge,
    ])
