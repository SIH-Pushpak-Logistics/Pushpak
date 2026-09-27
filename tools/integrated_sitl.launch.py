"""Source-workspace launch: v2 SITL + PR27 perception + scout Zenoh adapter.

No takeoff unless explicitly requested. Build/source ROS packages and build the
release Rust publisher before launching this file (see INTEGRATION_AUDIT.md).
"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    root = Path(__file__).resolve().parents[1]
    sitl = Path(get_package_share_directory('drone_description')) / 'launch/sitl_bringup.launch.py'
    telemetry = root / 'hitl/zenoh_publisher/launch/zenoh_publisher.launch.py'
    return LaunchDescription([
        DeclareLaunchArgument('auto_takeoff', default_value='false'),
        DeclareLaunchArgument('device', default_value='cuda:0'),
        DeclareLaunchArgument('connect_endpoints', default_value='[""]'),
        DeclareLaunchArgument('listen_endpoints', default_value='[""]'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(sitl)), launch_arguments={
            'use_sim_time': 'true', 'drone_id': '1',
            'auto_takeoff': LaunchConfiguration('auto_takeoff'),
        }.items()),
        Node(package='navigation_brain', executable='perception_node', output='screen',
             parameters=[{'use_sim_time': True, 'device': LaunchConfiguration('device'),
                          'model_path': str(root / 'yolov8n.pt')}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(telemetry)), launch_arguments={
            'use_sim_time': 'true', 'drone_id': '1', 'pose_source': 'odometry',
            'connect_endpoints': LaunchConfiguration('connect_endpoints'),
            'listen_endpoints': LaunchConfiguration('listen_endpoints'),
        }.items()),
    ])
