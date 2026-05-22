from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_dir = PathJoinSubstitution([FindPackageShare('ut_automata'), 'config'])
    hokuyo_launch = PathJoinSubstitution([
        FindPackageShare('ut_automata'),
        'launch',
        'hokuyo_10lx.launch.py',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_gui',
            default_value='true',
            description='Enable to start up the gui',
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(hokuyo_launch)),
        Node(
            package='ut_automata',
            executable='vesc_driver',
            name='vesc_driver',
            arguments=['--config_dir', config_dir],
            output='screen',
            respawn=True,
            respawn_delay=5.0,
        ),
        Node(
            package='ut_automata',
            executable='websocket',
            name='websocket',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),
        Node(
            package='ut_automata',
            executable='joystick',
            name='joystick',
            arguments=['--config_dir', config_dir],
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),
        Node(
            package='ut_automata',
            executable='gui',
            name='gui',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_gui')),
        ),
    ])
