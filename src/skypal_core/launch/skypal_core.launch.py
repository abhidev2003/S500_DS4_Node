import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Determine the context (simulation vs hardware)
    is_sim_arg = DeclareLaunchArgument(
        'is_sim',
        default_value='true',
        description='Running in simulation or real hardware'
    )
    is_sim = LaunchConfiguration('is_sim')

    # MAVROS Connection String:
    # Sim: connect to local SITL on loopback
    # Real: connect to RJ45 ethernet link
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='udp://127.0.0.1:14550@14555',
        description='MAVROS FCU URL (Use udp://192.168.100.2:14550@192.168.100.1:14550 for real drone)'
    )
    fcu_url = LaunchConfiguration('fcu_url')

    # MAVROS Node bridging MAVLink over the FCU URL to ROS 2 topics over Tailscale DDS
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        namespace='mavros',
        output='screen',
        parameters=[{
            'fcu_url': fcu_url,
            'gcs_url': '',
            'target_system_id': 1,
            'target_component_id': 1,
            'fcu_protocol': 'v2.0',
        }]
    )

    # Core Logic Node handling Failsafe
    heart_node = Node(
        package='skypal_core',
        executable='heart_node',
        name='heart_node',
        output='screen',
        parameters=[{'is_sim': is_sim}]
    )

    return LaunchDescription([
        is_sim_arg,
        fcu_url_arg,
        mavros_node,
        heart_node
    ])
