import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    common = {
        'use_sim_time': use_sim_time,
        'drone_id': ParameterValue(LaunchConfiguration('drone_id'), value_type=int),
    }
    ekf_config = os.path.join(get_package_share_directory('navigation_brain'), 'config', 'ekf_15state.yaml')

    onboard_nodes = [
        Node(package='navigation_brain', executable=exe, name=exe,
             output='screen', parameters=[common])
        for exe in ['altimeter_node', 'vio_bridge_node']
    ]
    ekf = Node(package='robot_localization', executable='ekf_node', name='ekf_node',
               output='screen', parameters=[ekf_config, {'use_sim_time': use_sim_time}])
    brain = Node(package='pushpak_brain', executable='pushpak_brain', name='pushpak_brain',
                 output='screen', parameters=[common])
    handshake = Node(package='navigation_brain', executable='arm_takeoff_handshake',
                     name='arm_takeoff_handshake', output='screen',
                     parameters=[{'use_sim_time': use_sim_time}],
                     condition=IfCondition(LaunchConfiguration('auto_takeoff')))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('drone_id'),
        DeclareLaunchArgument('auto_takeoff', default_value='false'),
        ekf,
        brain,
        handshake,
    ] + onboard_nodes)
