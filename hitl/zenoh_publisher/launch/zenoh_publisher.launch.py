"""Launch the rig adapter with the same topic contract in Tier 1 and Tier 3."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    script = str(Path(__file__).resolve().parents[1] / 'ros_adapter.py')
    arguments = {
        'use_sim_time': 'false',
        'mock_x_m': '0.0',
        'mock_y_m': '0.0',
        'mock_z_m': '1.0',
        'mock_yaw_deg': '0.0',
        'connect_endpoints': '[""]',
        'listen_endpoints': '[""]',
    }
    declarations = [DeclareLaunchArgument('pose_source',
                    description='Required: odometry or explicit mock pose')]
    declarations += [DeclareLaunchArgument(name, default_value=value)
                    for name, value in arguments.items()]
    command = ['python3', script, '--ros-args', '-p', 'drone_id:=2']
    for name in ('pose_source', *arguments):
        command.extend(['-p', [name + ':=', LaunchConfiguration(name)]])
    return LaunchDescription(declarations + [
        ExecuteProcess(cmd=command, output='screen'),
    ])
