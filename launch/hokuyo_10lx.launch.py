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
            'serial_port': '',
            'serial_baud': 115200,
            'frame_id': 'laser',
            'calibrate_time': False,
            'publish_intensity': True,
            'publish_multiecho': False,
            'angle_min': -2.25,
            'angle_max': 2.25,
        }],
    ))

    return ld
