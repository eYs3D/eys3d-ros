#!/usr/bin/env python3
"""Preview all three eYs3D camera models together in rviz, no hardware.

    ros2 launch eys3d_camera three_models.launch.py

Loads urdf/eys3d_three_models.urdf.xacro — G100P, G62 and R77 side by
side under a shared base_link — into robot_state_publisher and opens
rviz. Use it to inspect and compare the meshes and mounting-hole frames.
For a single model use display_model.launch.py; for the live camera plus
its model use the per-model driver launches (urdf:=true, the default).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def generate_launch_description():
    pkg_share = get_package_share_directory('eys3d_camera')
    xacro_file = os.path.join(pkg_share, 'urdf', 'eys3d_three_models.urdf.xacro')
    rviz_cfg = os.path.join(pkg_share, 'rviz', 'eys3d_camera_model.rviz')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro ', xacro_file]), value_type=str),
            }],
        ),
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            output='log',
            arguments=['-d', rviz_cfg],
        ),
    ])
