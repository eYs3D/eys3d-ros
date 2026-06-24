#!/usr/bin/env python3
"""Launch one G100+ and one R77 in a single ROS 2 process.

The two modules have distinct USB PIDs (0x0181 and 0x0180), so the
driver's auto-PID selection resolves each camera uniquely without an
explicit usb_port. For production wiring, pin each instance to a
specific USB socket via the usb_port argument so the same physical
port always maps to the same camera_name across reboots.

    ros2 launch eys3d_camera examples/G100P_plus_R77.launch.py

Per-camera topics:
    /G100P_1/{left_color, depth_image, pointcloud, ...}
    /R77_1/{left_color, depth_image, pointcloud, ...}
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def _camera(model: str, camera_name: str, mode_id: str, usb_port: str = ''):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('eys3d_camera'),
            'launch', 'camera.launch.py')),
        launch_arguments={
            'model':              model,
            'mode_id':            mode_id,
            'camera_name':        camera_name,
            'usb_port':           usb_port,
            'rviz':               'false',
            'log':                'sdk',
        }.items(),
    )


def generate_launch_description():
    return LaunchDescription([
        # G100+ at video mode 1 (1280x720 interleave, SDK 30 fps).
        # R77 at video mode 2 (1280x920 color, 640x460 depth, 30 fps).
        # The two PIDs differ, so auto-PID selection is sufficient for this
        # example. For production wiring set usb_port explicitly, e.g.
        # '2-3:1.0' for G100+ and '1-3:1.0' for R77.
        _camera('G100P', 'G100P_1', mode_id='3', usb_port=''),
        _camera('R77',   'R77_1',   mode_id='2', usb_port=''),
    ])
