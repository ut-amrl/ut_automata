#!/usr/bin/env python3
"""ROS2 launch file port of the ROS1 `logging.launch`.

Starts the `logger` node from `ut_automata` with an `--out_dir` argument.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    out_dir = LaunchConfiguration('out_dir')

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'out_dir', default_value='/home/amrl_user/bagfiles/',
        description='Directory where logger will write bagfiles'))

    ld.add_action(Node(
        package='ut_automata',
        executable='logger',
        name='logger',
        output='screen',
        arguments=['--out_dir', out_dir],
        respawn=False,
    ))

    return ld
