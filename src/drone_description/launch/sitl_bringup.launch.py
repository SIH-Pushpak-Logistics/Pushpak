import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    drone_desc_dir = get_package_share_directory('drone_description')
    drone_bringup_dir = get_package_share_directory('drone_bringup')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    xacro_file = os.path.join(drone_desc_dir, 'urdf', 'drone.urdf.xacro')

    # 1. Enforce the Global Time Domain
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 2. Boot the Transform Tree
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file]), 
            'use_sim_time': use_sim_time
        }]
    )

    # 3. Boot Gazebo Physics
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')]),
        launch_arguments={'pause': 'false', 'use_sim_time': use_sim_time}.items()
    )
    
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(gazebo_ros_dir, 'launch', 'gzclient.launch.py')])
    )

    # 4. Inject the Physical Drone Model (Delayed to let Gazebo boot)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'swarm_drone', '-z', '0.2'],
        output='screen'
    )

    # 5. Boot the Logic Brains (Delayed to let the drone spawn)
    swarm_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(drone_bringup_dir, 'launch', 'swarm_bringup.launch.py')]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        robot_state_publisher,
        gazebo_server,
        gazebo_client,
        TimerAction(period=3.0, actions=[spawn_entity]),
        TimerAction(period=6.0, actions=[swarm_bringup])
    ])