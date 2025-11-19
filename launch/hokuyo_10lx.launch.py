#!/usr/bin/env python3
"""ROS2 launch file port of the ROS1 `hokuyo_10lx.launch`.

This starts the `urg_node` from the `urg_node` package with parameters matching
the original ROS1 launch.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(Node(
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node',
        output='screen',
        parameters=[{
            'ip_address': '192.168.0.10',
            'ip_port': 10940,
            'serial_port': '',
            'serial_baud': 115200,
            'laser_frame_id': 'laser',
            'angle_max': 2.35619,
            'angle_min': -2.35619,
            'publish_intensity': False,
            'publish_multiecho': False,
            'calibrate_time': False,
            'default_user_latency': 0.0,
            'diagnostics_tolerance': 0.05,
            'diagnostics_window_time': 5.0,
            'error_limit': 4,
            'get_detailed_status': False,
            'cluster': 1,
            'skip': 1,
        }],
    ))


    return ld
