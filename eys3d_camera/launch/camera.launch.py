#!/usr/bin/env python3
"""Generic eys3d_camera launch for any supported eYs3D depth camera.

Examples:
    # G100+ at its default mode
    ros2 launch eys3d_camera camera.launch.py model:=G100P mode_id:=1

    # R77
    ros2 launch eys3d_camera camera.launch.py model:=R77 mode_id:=1

    # Pin a specific camera by USB topology path
    ros2 launch eys3d_camera camera.launch.py model:=G100P usb_port:=2-3:1.0

    # Swap the filter tuning profile (enables stay separate)
    ros2 launch eys3d_camera camera.launch.py model:=G100P \\
        spatial_filter:=true temporal_filter:=true \\
        filter_profile:=default

Prefer the per-model shortcuts (G100P.launch.py / R77.launch.py /
G62.launch.py) for everyday use; this generic launch is provided for
scripting and multi-camera setups.

Filter tuning split:
    * enable switches (spatial_filter, temporal_filter, hole_filling,
      colored_pointcloud) stay on the launch command line — they
      decide whether a filter stage runs at all.
    * tuning values (alpha / delta / magnitude / holes_fill /
      persistence) live in `cfg/filter_profiles/<filter_profile>.yaml`.
      Copy `default.yaml` and edit individual fields to author a
      custom profile. Absent fields fall back to the node's compiled
      defaults.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, OpaqueFunction,
                            RegisterEventHandler)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.event_handlers import OnProcessIO
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('model', default_value='G100P',
                              description='Camera model. Catalogue file: launch/video_modes/<model>.yaml.'),
        DeclareLaunchArgument('mode_id', default_value='1',
                              description='Mode index inside the catalogue.'),
        DeclareLaunchArgument('camera_name', default_value='camera',
                              description='ROS namespace and frame-id prefix for this camera.'),
        DeclareLaunchArgument('dev_serial_number', default_value='',
                              description='Bind to a specific camera by serial-number substring.'),
        DeclareLaunchArgument('usb_port', default_value='',
                              description='Bind to a specific camera by USB topology path '
                                          '(e.g. "2-3:1.0"). Stable across reboots.'),
        DeclareLaunchArgument('depth_minimum_mm', default_value='-1',
                              description='PointCloud2 near clip in millimetres. '
                                          '-1 = default; positive integer to override.'),
        DeclareLaunchArgument('depth_maximum_mm', default_value='-1',
                              description='PointCloud2 far clip in millimetres. '
                                          '-1 = default; positive integer to override.'),
        DeclareLaunchArgument('colored_pointcloud', default_value='false',
                              description='Publish XYZRGB PointCloud2 (point_step=16) by sampling '
                                          'the latest decoded left-color frame at each valid '
                                          'depth pixel. Falls back to XYZ-only on D-only modes.'),
        DeclareLaunchArgument('spatial_filter', default_value='false',
                              description='Enable the spatial filter '
                                          '(4-direction edge-aware IIR). '
                                          'Tuning lives in the active filter_profile YAML.'),
        DeclareLaunchArgument('temporal_filter', default_value='false',
                              description='Enable the temporal filter '
                                          '(alpha-blend + persistence on the active depth '
                                          'raster). Runtime-tunable via `ros2 param set`. '
                                          'Tuning lives in the active filter_profile YAML.'),
        DeclareLaunchArgument('hole_filling', default_value='0',
                              description='Hole filling mode. '
                                          '0=off; 1=fill_from_left; '
                                          '2=farthest_from_around (try first when enabling); '
                                          '3=nearest_from_around.'),
        DeclareLaunchArgument('filter_profile', default_value='default',
                              description='Tuning profile name (resolves to '
                                          'cfg/filter_profiles/<name>.yaml). Seeds all '
                                          'post-processing tuning values; runtime-tunable '
                                          'via `ros2 param set`.'),
        DeclareLaunchArgument('diagnostics_rate_hz', default_value='1.0',
                              description='Publish rate for /diagnostics (0 = disabled).'),
        DeclareLaunchArgument('ir_intensity', default_value='-1',
                              description='IR projector level. '
                                          '-1 = per-PID default (G100+/R77 = 3, G62 = 60); '
                                          '0 = projector off; '
                                          'positive integer = raw level (clamped to FW max).'),
        DeclareLaunchArgument('log', default_value='all',
                              description='Terminal output level. '
                                          'all = full RCLCPP + SDK output (default); '
                                          'sdk = suppress RCLCPP, keep SDK printf; '
                                          'close = redirect everything to a per-process '
                                          'log file (terminal silent).'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Open the bundled RViz layout on launch. '
                                          'Default false here; the per-model launches default to true.'),
        DeclareLaunchArgument('rviz_config', default_value='',
                              description='Override path to the .rviz file. '
                                          'Empty = pick automatically from `model` '
                                          '(eys3d_camera_<MODEL>.rviz).'),
    ]

    pkg_share = get_package_share_directory('eys3d_camera')
    dm_quality_cfg_dir = os.path.join(pkg_share, 'cfg', 'DM_Quality_Cfg')

    omp_env = {
        'OMP_WAIT_POLICY': 'PASSIVE',
        'GOMP_SPINCOUNT':  '0',
        'OMP_NUM_THREADS': '4',
    }

    # Everything below is built inside _setup so that filter_profile (and
    # the rviz layout selector) can resolve their LaunchConfiguration into
    # real strings via context.perform(). Foxy's PathJoinSubstitution does
    # not accept nested-substitution items, so the YAML path is composed
    # in plain Python here.
    def _setup(context):
        profile = LaunchConfiguration('filter_profile').perform(context)
        filter_profile_yaml = os.path.join(
            pkg_share, 'cfg', 'filter_profiles', f'{profile}.yaml')

        param_dict = {
            'camera_name':         LaunchConfiguration('camera_name'),
            'model':               LaunchConfiguration('model'),
            'mode_id':             LaunchConfiguration('mode_id'),
            'dev_serial_number':   LaunchConfiguration('dev_serial_number'),
            'usb_port':            LaunchConfiguration('usb_port'),
            'depth_minimum_mm':    LaunchConfiguration('depth_minimum_mm'),
            'depth_maximum_mm':    LaunchConfiguration('depth_maximum_mm'),
            'colored_pointcloud':  LaunchConfiguration('colored_pointcloud'),
            'spatial_filter':      LaunchConfiguration('spatial_filter'),
            'temporal_filter':     LaunchConfiguration('temporal_filter'),
            'hole_filling':        LaunchConfiguration('hole_filling'),
            'diagnostics_rate_hz': LaunchConfiguration('diagnostics_rate_hz'),
            'dm_quality_cfg_dir':  dm_quality_cfg_dir,
            'ir_intensity':        LaunchConfiguration('ir_intensity'),
        }

        # Three Node variants below; exactly one runs, selected by the
        # 'log' argument. They differ only in output handling and rcl
        # log-level. The profile YAML is loaded before param_dict so any
        # explicit launch override wins over the profile baseline.
        common_kwargs = dict(
            package='eys3d_camera',
            executable='camera_node',
            name='eys3d_camera',
            namespace=LaunchConfiguration('camera_name'),
            additional_env=omp_env,
            parameters=[filter_profile_yaml, param_dict],
        )

        # log=all : full disclosure. RCLCPP at default INFO level plus SDK
        # stdout — every startup banner, mode-table dump, and per-stream
        # gating notice reaches the terminal. Use for first-run smoke tests
        # and bug reports.
        camera_node_all = Node(
            output='screen',
            emulate_tty=True,
            condition=LaunchConfigurationEquals('log', 'all'),
            **common_kwargs,
        )

        # log=sdk : essential disclosure. SDK stdout reaches the terminal
        # (device-open banner, mode application, FW version), RCLCPP WARN
        # and ERROR reach the terminal (failure paths, watchdog events),
        # RCLCPP INFO / DEBUG are suppressed. Default for routine
        # production use.
        camera_node_sdk = Node(
            output='screen',
            emulate_tty=False,
            arguments=['--ros-args', '--log-level', 'WARN'],
            condition=LaunchConfigurationEquals('log', 'sdk'),
            **common_kwargs,
        )

        # log=close : terminal silent. All output goes to the per-process
        # log file under ~/.ros/log/<run>/. Use for production where the
        # camera is one of many nodes and stdout noise must be contained.
        camera_node_close = Node(
            output='own_log',
            emulate_tty=False,
            arguments=['--ros-args', '--log-level', 'FATAL'],
            condition=LaunchConfigurationEquals('log', 'close'),
            **common_kwargs,
        )

        # rviz2 is held back until camera_node prints "EYS3D_CAMERA_READY"
        # on stdout (emitted at the first depth frame) to avoid a known
        # startup race on Foxy.
        rviz_cfg = LaunchConfiguration('rviz_config').perform(context)
        if not rviz_cfg:
            model = LaunchConfiguration('model').perform(context)
            rviz_cfg = os.path.join(pkg_share, 'rviz',
                                    f'eys3d_camera_{model}.rviz')
        rviz_node = Node(
            package='rviz2', executable='rviz2', name='rviz2',
            output='log',
            arguments=['-d', rviz_cfg],
        )
        rviz_fired = [False]
        def _on_io(event):
            if rviz_fired[0]:
                return None
            text = event.text
            if isinstance(text, (bytes, bytearray, memoryview)):
                text = bytes(text)
            else:
                text = str(text).encode('utf-8', errors='ignore')
            if b'EYS3D_CAMERA_READY' in text:
                rviz_fired[0] = True
                return [rviz_node]
            return None

        rviz_handlers = [
            RegisterEventHandler(
                OnProcessIO(target_action=cam, on_stdout=_on_io),
                condition=IfCondition(LaunchConfiguration('rviz')),
            )
            for cam in (camera_node_all, camera_node_sdk, camera_node_close)
        ]

        return [camera_node_all, camera_node_sdk, camera_node_close,
                *rviz_handlers]

    return LaunchDescription(args + [OpaqueFunction(function=_setup)])
