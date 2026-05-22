from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'localize',
            default_value='false',
            description='Publish ground-truth localization from the simulator',
        ),
        Node(
            package='ut_automata',
            executable='simulator',
            name='ut_automata_simulator',
            arguments=['--localize', LaunchConfiguration('localize')],
            output='screen',
        ),
        Node(
            package='ut_automata',
            executable='websocket',
            name='websocket',
            output='screen',
        ),
    ])
