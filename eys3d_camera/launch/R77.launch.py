#!/usr/bin/env python3
"""Launch shortcut for the eYs3D R77 depth camera.

Equivalent to:
    ros2 launch eys3d_camera camera.launch.py model:=R77 camera_name:=R77_1

The common camera.launch.py arguments are forwarded; `diagnostics_rate_hz`
is available only on `camera.launch.py`.

mode_id defaults to -1 (auto): the driver probes the negotiated USB link and
opens that model's signature default mode for it. Refer to
launch/video_modes/R77.yaml for the full mode catalog.

Filter tuning is loaded from `cfg/filter_profiles/<filter_profile>.yaml`;
the enable switches stay on the launch command line.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    args = [
        # Identity / binding
        DeclareLaunchArgument('mode_id',           default_value='-1',
                              description='Mode index from launch/video_modes/R77.yaml; -1 = auto (signature default for the negotiated USB link).'),
        DeclareLaunchArgument('camera_name',       default_value='R77_1',
                              description='ROS namespace and frame-id prefix.'),
        DeclareLaunchArgument('dev_serial_number', default_value='',
                              description='Bind to a specific camera by serial-number substring.'),
        DeclareLaunchArgument('usb_port',          default_value='',
                              description='Bind to a specific camera by USB topology path (e.g. "2-3:1.0").'),

        # Hardware / range
        DeclareLaunchArgument('ir_value',          default_value='-1',
                              description='-1 = use per-model default (3). '
                                          '0 = projector off. 1-6 = raw FW level.'),
        DeclareLaunchArgument('depth_near_mm',     default_value='-1',
                              description='-1 = use default (200 mm); positive integer to override.'),
        DeclareLaunchArgument('depth_far_mm',      default_value='-1',
                              description='-1 = use default (1500 mm); positive integer to override.'),

        # Post-processing enables
        DeclareLaunchArgument('colored_pointcloud', default_value='false',
                              description='Publish XYZRGB PointCloud2 sampled from the latest colour frame.'),
        DeclareLaunchArgument('spatial_filter',     default_value='false',
                              description='Enable the spatial filter (4-direction edge-aware IIR).'),
        DeclareLaunchArgument('temporal_filter',    default_value='false',
                              description='Enable the temporal filter (alpha-blend + persistence).'),
        DeclareLaunchArgument('hole_filling',       default_value='0',
                              description='Hole filling mode. '
                                          '0=off; 1=fill_from_left; '
                                          '2=farthest_from_around (try first when enabling); '
                                          '3=nearest_from_around.'),
        DeclareLaunchArgument('filter_profile',     default_value='default',
                              description='Tuning profile filename (without .yaml) under '
                                          'cfg/filter_profiles/. Seeds all post-processing '
                                          'tuning values at startup.'),

        # Output / diagnostics
        DeclareLaunchArgument('selfcal_enable', default_value='false',
                              description='Enable in-stream self-calibration.'),
        DeclareLaunchArgument('log',  default_value='sdk',
                              description='Terminal output level: all / sdk(default) / close. '
                                          'See camera.launch.py for details.'),
        DeclareLaunchArgument('urdf', default_value='true',
                              description='Publish the camera model (URDF) via '
                                          'robot_state_publisher. false = driver only.'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open the bundled RViz layout on launch.'),
    ]

    camera_launch = os.path.join(
        get_package_share_directory('eys3d_camera'),
        'launch', 'camera.launch.py')

    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch),
        launch_arguments={
            'model':              'R77',
            'mode_id':            LaunchConfiguration('mode_id'),
            'camera_name':        LaunchConfiguration('camera_name'),
            'dev_serial_number':  LaunchConfiguration('dev_serial_number'),
            'usb_port':           LaunchConfiguration('usb_port'),
            'ir_value':           LaunchConfiguration('ir_value'),
            'depth_near_mm':      LaunchConfiguration('depth_near_mm'),
            'depth_far_mm':       LaunchConfiguration('depth_far_mm'),
            'colored_pointcloud': LaunchConfiguration('colored_pointcloud'),
            'spatial_filter':     LaunchConfiguration('spatial_filter'),
            'temporal_filter':    LaunchConfiguration('temporal_filter'),
            'hole_filling':       LaunchConfiguration('hole_filling'),
            'filter_profile':     LaunchConfiguration('filter_profile'),
            'selfcal_enable':     LaunchConfiguration('selfcal_enable'),
            'log':                LaunchConfiguration('log'),
            'urdf':               LaunchConfiguration('urdf'),
            'rviz':               LaunchConfiguration('rviz'),
        }.items(),
    )

    return LaunchDescription(args + [include])
