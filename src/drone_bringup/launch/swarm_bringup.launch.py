import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Resolve Package Paths
    drone_bringup_dir = get_package_share_directory('drone_bringup')
    bridge_config_path = os.path.join(drone_bringup_dir, 'config', 'bridge.yaml')

    # 2. The ROS-GZ Bridge Node
    # This node consumes the bridge.yaml file and translates gz.msgs to sensor_msgs
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config_path,
            'expand_gz_topic_names': True
        }],
        output='screen'
    )

    # 3. Build and Return the Execution Graph
    return LaunchDescription([
        bridge_node
    ])