import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. Define the system-wide arguments
    drone_id_arg = DeclareLaunchArgument(
        'drone_id',
        default_value='drone_00',
        description='Unique identifier for the swarm drone'
    )
    
    drone_id = LaunchConfiguration('drone_id')

    # 2. Architect the Node Bringup Sequence
    # This enforces that all nodes share the exact same drone_id namespace
    
    vision_nav_node = Node(
        package='navigation_brain',
        executable='vision_nav_node',
        name='vision_nav',
        parameters=[{'drone_id': drone_id}],
        output='screen'
    )
    
    landing_state_node = Node(
        package='navigation_brain',
        executable='landing_state_node', # Shivam's future executable
        name='landing_state',
        parameters=[{'drone_id': drone_id}],
        output='screen'
    )

    state_machine_node = Node(
        package='navigation_brain',
        executable='state_machine_node',
        name='master_state_machine',
        parameters=[{'drone_id': drone_id}],
        output='screen'
    )

    # 3. Execute the Swarm
    return LaunchDescription([
        drone_id_arg,
        vision_nav_node,
        landing_state_node,
        state_machine_node
    ])