import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
from os.path import join

def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_ros_gz_rbot = get_package_share_directory('urdf_description_oct')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    robot_description_file = os.path.join(pkg_ros_gz_rbot, 'urdf', 'October_2026_URDF.xacro')
    ros_gz_bridge_config = os.path.join(pkg_ros_gz_rbot, 'config', 'ros_gz_bridge_gazebo.yaml')
    world_file = os.path.join(pkg_ros_gz_rbot, 'worlds', 'turtlebot3_world.world')

    # Ensure Gazebo Sim can find models referenced in the world (e.g. turtlebot3_world)
    tb3_models_path = os.path.join(pkg_tb3_gazebo, 'models')
    gz_sim_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    combined_resource_path = f"{tb3_models_path}:{gz_sim_resource_path}" if gz_sim_resource_path else tb3_models_path

    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=combined_resource_path
    )

    robot_description_config = xacro.process_file(robot_description_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": f"-r -v 4 {world_file}"}.items()
    )

    spawn_robot = TimerAction(
        period=5.0,  
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                "-topic", "/robot_description",
                "-name", "October_2026_URDF",
                "-allow_renaming", "false",  # prevents "_1" duplicate
                "-x", "-2.0",
                "-y", "-0.5",
                "-z", "0.32",
                "-Y", "0.0"
            ],
            output='screen'
        )]
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': ros_gz_bridge_config}],
        output='screen'
    )

    scan_filter = Node(
        package='urdf_description_oct',
        executable='scan_filter',
        name='scan_filter',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gazebo_resource_path,
        gazebo,
        spawn_robot,
        ros_gz_bridge,
        robot_state_publisher,
        scan_filter,
    ])
