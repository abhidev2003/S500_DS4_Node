import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # The URL where MAVROS talks to the Pixhawk
    fcu_url = '/dev/ttyACM0:115200'
    
    # The URL where MAVROS sends telemetry to QGroundControl
    # Base Station PC Tailscale IP = 100.122.20.128
    gcs_url = 'udp://@100.122.20.128:14550'

    return LaunchDescription([
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            output='screen',
            parameters=[{
                'fcu_url': fcu_url,
                'gcs_url': gcs_url,
                'target_system_id': 1,
                'target_component_id': 1,
                'fcu_protocol': 'v2.0',
                'plugin_allowlist': ['sys_status', 'sys_time', 'command', 'setpoint_velocity', 'rc_io', 'imu', 'local_position', 'global_position']
            }]
        )
    ])
