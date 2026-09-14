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
    sway_params_path = os.path.join(
        get_package_share_directory('navigation_brain'), 'config', 'sway_params.yaml')

    common = {'use_sim_time': True, 'drone_id': 'drone_00'}

    brain_nodes = [
        Node(package='navigation_brain', executable=exe, name=exe,
             output='screen', parameters=[common])
        for exe in ['altimeter_node', 'mission_node', 'vision_nav_node', 'landing_state_node', 'state_machine_node']
    ]

    brain_nodes.append(
        Node(package='navigation_brain', executable='anti_sway_filter',
             name='anti_sway_filter', output='screen',
             parameters=[sway_params_path, common])
    )

    return LaunchDescription([bridge_node] + brain_nodes)