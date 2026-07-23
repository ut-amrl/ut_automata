#!/usr/bin/env python3
"""Auto-selecting lidar bringup for the roboracer car.

Two lidars are supported, and only one is fitted at a time:

  * Slamtec RPLIDAR C1 - USB (CP2102N UART bridge -> /dev/ttyUSB0), 460800 baud,
    driven by ``rplidar_ros`` (``rplidar_node``).
  * Hokuyo UST-10LX    - Ethernet (192.168.0.10:10940), driven by ``urg_node``.

Which driver starts is decided here at launch time from what's plugged in:
a USB serial device present -> RPLIDAR; otherwise -> Hokuyo. If both happen to be
connected the RPLIDAR wins. Either way the driver publishes sensor_msgs/LaserScan
on ``/scan`` with ``frame_id: laser``, so nothing downstream (TF, nav2, leg
detector) changes.

Note on detection: airfield maps host devices into the container at container
start (there is no hotplug), and ``/dev/ttyUSB0`` is only mapped when it's
actually present on the host (package_exec skips absent devices). So this glob
sees the RPLIDAR exactly when it is physically connected; with the Hokuyo there
is no ttyUSB node and we fall through to urg_node.
"""

import glob

from launch import LaunchDescription
from launch_ros.actions import Node


def _rplidar_serial_port():
    """Return the RPLIDAR's serial device, or None if no USB serial is present.

    Nothing else on this car enumerates as /dev/ttyUSB* (the VESC is ttyACM0),
    so a ttyUSB node is the RPLIDAR C1's CP2102N USB-UART bridge.
    """
    ports = sorted(glob.glob('/dev/ttyUSB*'))
    return ports[0] if ports else None


def generate_launch_description():
    ld = LaunchDescription()

    rplidar_port = _rplidar_serial_port()

    if rplidar_port is not None:
        # Slamtec RPLIDAR C1 over USB serial.
        ld.add_action(Node(
            package='rplidar_ros',
            # We build rplidar_ros from Slamtec's ros2 branch (see the
            # dependencies/xplatform/rplidar_ros.yaml manifest) because the apt
            # package's older SDK can't start a scan on the C1. That source
            # build names the executable 'rplidar_node' (the apt build called it
            # 'rplidar_composition'); same node/params, different exe name.
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': rplidar_port,
                'serial_baudrate': 460800,   # C1 = 460800 (A-series = 115200, S2/S3 = 1000000)
                'frame_id': 'laser',         # match the Hokuyo's frame so TF is unchanged
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard',     # C1 default (matches Slamtec's rplidar_c1_launch.py)
            }],
        ))
    else:
        # Hokuyo UST-10LX over Ethernet (192.168.0.10:10940). Params preserved
        # verbatim from the original hokuyo_10lx.launch.py (ROS1 port).
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
