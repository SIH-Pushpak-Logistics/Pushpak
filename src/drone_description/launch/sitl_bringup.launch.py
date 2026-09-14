import os
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    drone_desc_dir = get_package_share_directory('drone_description')
    drone_bringup_dir = get_package_share_directory('drone_bringup')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(drone_desc_dir, 'urdf', 'drone.urdf.xacro')
    mavros_config = os.path.join(drone_bringup_dir, 'config', 'apm_config.yaml')
    
    # Path to the parameters mapped via docker-compose volume
    ardupilot_param_file = '/workspace/firmware/ardupilot_config/base_iris.param'

    # 1. Enforce the Global Time Domain
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 2. Boot the Transform Tree
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str),
            'use_sim_time': use_sim_time
        }]
    )

    # 3. Boot Gazebo Harmonic
    world_file = os.path.join(drone_desc_dir, 'worlds', 'swarm.sdf')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': '-s -r -v 4 ' + world_file}.items()
    )

    # 4. Inject the Physical Drone Model via Harmonic's Spawner
    spawn_entity_harmonic = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'swarm_drone', '-z', '0.2'],
        output='screen'
    )

    # 5. Boot the Cerebellum (ArduPilot SITL)
    # The -S flag runs SITL, -I0 is instance 0, and --model gazebo-iris connects to your plugin
    ardupilot_sitl = ExecuteProcess(
        cmd=['/firmware/ardupilot/build/sitl/bin/arducopter', 
             '-S', '-I0', 
             '--model', 'JSON:127.0.0.1', 
             '--defaults', ardupilot_param_file],
        output='screen'
    )

    # 6. Boot the Spine (MAVROS)
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        parameters=[
            mavros_config,
            {
                'fcu_url': 'tcp://127.0.0.1:5760',
                'gcs_url': 'udp://@localhost:14550', 
                'use_sim_time': use_sim_time
            }
        ]
    )


    # 7. Boot the Logic Brains
    swarm_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(drone_bringup_dir, 'launch', 'swarm_bringup.launch.py')]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 8. Strict Deterministic Execution Handoff
    spawn_exit_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_harmonic,
            on_exit=[ardupilot_sitl, swarm_bringup] 
        )
    )

    # 9. Wait for SITL to actually start before starting the MAVROS timer
    sitl_start_event = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ardupilot_sitl,
            on_start=[
                TimerAction(
                    period=20.0,
                    actions=[mavros_node]
                )
            ]
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        robot_state_publisher,
        gz_sim,
        spawn_entity_harmonic,
        spawn_exit_event,
        sitl_start_event
    ])