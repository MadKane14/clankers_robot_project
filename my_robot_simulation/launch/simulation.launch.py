import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get package directories
    pkg_my_robot_simulation = get_package_share_directory('my_robot_simulation')
    pkg_gazebo_robot = get_package_share_directory('gazebo_differential_drive_robot')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # --- THE FIX: Tell Gazebo where to find your package's meshes/models ---
    workspace_share_dir = os.path.dirname(pkg_gazebo_robot)
    set_env_vars_resources = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=workspace_share_dir
    )

    # Path to your custom world
    world_file = os.path.join(pkg_my_robot_simulation, 'worlds', 'custom_world1.sdf')

    # Launch Gazebo Harmonic with the custom world AND verbose logging (-v 4)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v 4 {world_file}'}.items()
    )

    # Spawn the robot with custom coordinates to avoid clipping into walls
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_robot, 'launch', 'robot.launch.py') 
        ),
        launch_arguments={
            'x': '-8.0',  # Moved 8 meters back
            'y': '2.0',   # Moved 2 meters left
            'z': '0.5'
        }.items()
    )

    return LaunchDescription([
        set_env_vars_resources,
        gazebo,
        spawn_robot
    ])