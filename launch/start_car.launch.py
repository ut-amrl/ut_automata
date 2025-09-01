#!/usr/bin/env python3
"""ROS2 launch file port of the ROS1 `start_car.launch`.

This starts the hokuyo include (assumed converted to a ROS2 python launch),
and launches the vesc_driver, websocket, joystick and gui nodes. The gui
node is controlled by the `start_gui` launch argument.

Assumptions:
- A ROS2-compatible `hokuyo_10lx` launch exists at
  `launch/hokuyo_10lx.launch.py` inside the `ut_automata` package.
- The package `ut_automata` exposes executables named
  `vesc_driver`, `websocket`, `joystick`, and `gui`.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    start_gui = LaunchConfiguration('start_gui')

    # package share and paths
    pkg_share = get_package_share_directory('ut_automata')
    config_dir = os.path.join(pkg_share, 'config')
    hokuyo_launch = os.path.join(pkg_share, 'launch', 'hokuyo_10lx.launch.py')

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'start_gui', default_value='true', description='Enable to start up the gui'))

    # Include hokuyo launch (assumes a ROS2 python launch exists at the path above)
    if os.path.exists(hokuyo_launch):
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hokuyo_launch)
        ))
    else:
        # If the hokuyo launch isn't present, we still continue but users should
        # convert or add the hokuyo launch file at the expected location.
        pass

    # vesc_driver node
    ld.add_action(Node(
        package='ut_automata',
        executable='vesc_driver',
        name='vesc_driver',
        output='screen',
        arguments=['--config_dir', config_dir],
        respawn=True,
        respawn_delay=5.0,
    ))

    # websocket node
    ld.add_action(Node(
        package='ut_automata',
        executable='websocket',
        name='websocket',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    ))

    # joystick node
    ld.add_action(Node(
        package='ut_automata',
        executable='joystick',
        name='joystick',
        output='screen',
        arguments=['--config_dir', config_dir],
        respawn=True,
        respawn_delay=2.0,
    ))

    # gui node, only if start_gui is true
    ld.add_action(Node(
        package='ut_automata',
        executable='gui',
        name='gui',
        output='screen',
        respawn=False,
        condition=IfCondition(start_gui),
    ))

    return ld
