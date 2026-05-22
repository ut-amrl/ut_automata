from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='urg_node2',
            executable='urg_node2_node',
            name='urg_node',
            output='screen',
            parameters=[{
                'ip_address': '192.168.0.10',
                'serial_port': '',
                'serial_baud': 115200,
                'frame_id': 'laser',
                'calibrate_time': True,
                'publish_intensity': False,
                'publish_multiecho': False,
                'angle_min': -2.25,
                'angle_max': 2.25,
            }],
        ),
    ])
