#!/usr/bin/env python3
"""Two G100+ cameras in a single launch.

Same-model multi-camera deployments require an explicit binding hint per
instance — `dev_serial_number` (printed on the module label) or `usb_port`
(sysfs topology, e.g. `2-3:1.0`). The driver's auto-PID selection alone
cannot tell two G100+ modules apart.

Edit the two usb_port values below to match your wiring before launching:

    ros2 launch eys3d_camera examples/dual_G100P.launch.py

Per-camera topics:
    /G100P_left/{left_color, depth_image, pointcloud, ...}
    /G100P_right/{left_color, depth_image, pointcloud, ...}
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def _camera(camera_name: str, usb_port: str, mode_id: str = '7'):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('eys3d_camera'),
            'launch', 'camera.launch.py')),
        launch_arguments={
            'model':              'G100P',
            'mode_id':            mode_id,
            'camera_name':        camera_name,
            'usb_port':           usb_port,
            'rviz':               'false',
            'log':                'sdk',
        }.items(),
    )


def generate_launch_description():
    return LaunchDescription([
        # Replace the usb_port values below with the sysfs paths of the two
        # connected G100+ modules. To list connected eYs3D modules:
        #   ls -l /sys/class/video4linux/video*/device | grep 3438
        _camera('G100P_left',  usb_port='2-3:1.0'),
        _camera('G100P_right', usb_port='2-4:1.0'),
    ])
