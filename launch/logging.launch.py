from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'out_dir',
            default_value='/home/amrl_user/bagfiles/',
            description='Directory for rosbag2 output',
        ),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a', '-o', LaunchConfiguration('out_dir')],
            output='screen',
        ),
    ])
