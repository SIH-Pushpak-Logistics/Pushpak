#!/usr/bin/env python3
"""Final unarmed peer-2 tabletop rig launch."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    root = Path(__file__).resolve().parents[1]
    params = str(root / 'hitl/config/rig_params.yaml')
    mavros_base = str(root / 'src/drone_bringup/config/apm_config.yaml')
    ekf = str(root / 'src/navigation_brain/config/ekf_15state.yaml')
    adapter = str(root / 'hitl/zenoh_publisher/ros_adapter.py')

    params_file = LaunchConfiguration('params_file')
    tof_topic = LaunchConfiguration('tof_topic')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=params),
        DeclareLaunchArgument(
            'tof_topic', default_value='/mavros/distance_sensor/rangefinder_pub',
            description='sensor_msgs/Range input; remap for an external ToF driver'),

        Node(
            package='mavros', executable='mavros_node', name='mavros',
            output='screen', parameters=[mavros_base, params_file],
            remappings=[('/mavros/imu/data_raw', '/imu/raw')]),
        Node(
            package='v4l2_camera', executable='v4l2_camera_node',
            name='usb_camera', namespace='camera', output='screen',
            parameters=[params_file]),
        Node(
            package='navigation_brain', executable='altimeter_node',
            name='altimeter_node', output='screen', parameters=[params_file],
            remappings=[('/drone/rangefinder/raw', tof_topic)]),
        Node(
            package='robot_localization', executable='ekf_node',
            name='ekf_node', output='screen',
            parameters=[ekf, params_file]),
        Node(
            package='navigation_brain', executable='perception_node',
            name='perception_node', output='screen', parameters=[params_file]),
        ExecuteProcess(
            cmd=['python3', adapter, '--ros-args', '--params-file', params_file],
            output='screen'),
    ])
