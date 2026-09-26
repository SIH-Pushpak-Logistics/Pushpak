from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    common = {
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
        'drone_id': ParameterValue(LaunchConfiguration('drone_id'), value_type=int),
    }

    onboard_nodes = [
        Node(package='navigation_brain', executable=exe, name=exe,
             output='screen', parameters=[common])
        for exe in ['altimeter_node']
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('drone_id'),
    ] + onboard_nodes)
