#!/usr/bin/env python3
"""Preview a camera description in rviz, no hardware required.

    ros2 launch eys3d_camera display_model.launch.py model:=G100P
    ros2 launch eys3d_camera display_model.launch.py model:=G62 camera_name:=G62_left

Renders the model's mesh, the mounting-hole frames, and the base link.
With the driver running, the same description plugs the per-stream TF
tree published on /tf_static into your robot model — instantiate the
macro from urdf/eys3d_<MODEL>.urdf.xacro under your own parent link
instead (see the Frame ID section of the README).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('eys3d_camera')
    xacro_file = os.path.join(pkg_share, 'urdf', 'eys3d_camera.urdf.xacro')
    rviz_cfg = os.path.join(pkg_share, 'rviz', 'eys3d_camera_model.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='G100P',
                              description='G100P / G100Pi-as-G100P / G62 / R77'),
        DeclareLaunchArgument('camera_name', default_value='',
                              description='Defaults to <model>_1'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(Command([
                    'xacro ', xacro_file,
                    ' model:=', LaunchConfiguration('model'),
                    ' camera_name:=', LaunchConfiguration('camera_name'),
                ]), value_type=str),
            }],
        ),
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            output='log',
            arguments=['-d', rviz_cfg],
        ),
    ])
