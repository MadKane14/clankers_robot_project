import os
import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Define the robot's name and package name
    robot_name = "differential_drive_robot"
    package_name = "gazebo_differential_drive_robot"

    # Define launch arguments for initial pose
    x_arg = DeclareLaunchArgument(
        'x', default_value='0.0', description='Initial X position')
    y_arg = DeclareLaunchArgument(
        'y', default_value='0.0', description='Initial Y position')
    z_arg = DeclareLaunchArgument(
        'z', default_value='0.5', description='Initial Z position')
    roll_arg = DeclareLaunchArgument(
        'R', default_value='0.0', description='Initial Roll')
    pitch_arg = DeclareLaunchArgument(
        'P', default_value='0.0', description='Initial Pitch')
    yaw_arg = DeclareLaunchArgument(
        'Y', default_value='0.0', description='Initial Yaw')

    # Retrieve launch configurations
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('R')
    pitch = LaunchConfiguration('P')
    yaw = LaunchConfiguration('Y')

    # Set paths to Xacro model and configuration files
    robot_model_path = os.path.join(
        get_package_share_directory(package_name),
        'model',
        'robot.xacro'
    )

    gz_bridge_params_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'gz_bridge.yaml'
    )

    # Process the Xacro file to generate the URDF representation of the robot
    robot_description = xacro.process_file(robot_model_path).toxml()

    # Create a node to spawn the robot model in the Gazebo environment
    spawn_model_gazebo_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_name,
            '-topic', 'robot_description',
            '-x', x,
            '-y', y,
            '-z', z,
            '-R', roll,
            '-P', pitch,
            '-Y', yaw,
            '-allow_renaming', 'false'
        ],
        output='screen',
    )

    # Create a node to publish the robot's state based on its URDF description
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description, 'use_sim_time': True}
        ],
        output='screen'
    )

  # Create a node for the ROS-Gazebo bridge to handle message passing
    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': gz_bridge_params_path
        }],
        output='screen'
    )

    return LaunchDescription([
        x_arg,
        y_arg,
        z_arg,
        roll_arg,
        pitch_arg,
        yaw_arg,
        spawn_model_gazebo_node,
        robot_state_publisher_node,
        gz_bridge_node,
    ])