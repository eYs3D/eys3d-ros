#!/usr/bin/env python3
"""eys3d_camera G100+ composable launch.

Loads the G100+ camera in mode 1 (L'+D 1280x720 at 30 fps interleaved)
into a ComposableNodeContainer. The argument
surface covers the settings a deployment typically overrides
(camera identity, USB binding, intra-process comms) and
accepts a `params_file` so a deployment's full driver configuration
can be carried in a single YAML.

Add downstream subscribers as ComposableNode entries inside this
file so they share the container with CameraNode; with
`use_intra_process_comms:=true` each Image or PointCloud2 message
is delivered to those subscribers by pointer rather than copied
through DDS.

Topic delivery summary:
  * /tf_static is always published once with TRANSIENT_LOCAL
    durability and intra-process delivery explicitly disabled on
    that publisher, so it works on every supported ROS 2 distro.
  * /<camera_name>/{left_color/image_raw, depth/image_raw, depth/points,
    camera_info, diagnostics} use VOLATILE durability and follow
    the node-level IPC setting; with intra-process comms enabled
    the publishes hand a unique_ptr to same-container subscribers.

Usage:
  ros2 launch eys3d_camera G100P_composable.launch.py
  ros2 launch eys3d_camera G100P_composable.launch.py \\
      use_intra_process_comms:=true
  ros2 launch eys3d_camera G100P_composable.launch.py \\
      params_file:=/path/to/g100p.yaml
"""

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


# Camera identity baked into this launch. Switching to a different
# eYs3D model means duplicating the file and editing these three
# constants; the rest of the launch is reusable.
MODEL = 'G100P'
MODE_ID = 1
FILTER_PROFILE = 'default'


def _camera_parameter_list(context, filter_profile_yaml: str,
                           camera_params: dict) -> list:
    """Build the CameraNode parameters= list.

    Order matters — later entries override earlier ones. The filter
    profile YAML seeds the tuning defaults; the explicit launch
    arguments override those; an optional `params_file` YAML wins
    over both, allowing every driver setting to be carried in one
    file. Foxy's ComposableNode dispatch does not parse YAML file
    paths inline the way the Node action does, so the YAML is read
    here and merged into the parameter dict directly.
    """
    items: list = [filter_profile_yaml, camera_params]
    params_file = LaunchConfiguration('params_file').perform(context)
    if params_file:
        with open(params_file) as f:
            doc = yaml.safe_load(f) or {}
        # ROS 2 params YAML wraps values under `<node>:/ros__parameters:`.
        # `/**:` matches every node, so its ros__parameters block is the
        # canonical place for shared driver settings.
        extra = doc.get('/**', {}).get('ros__parameters', {})
        items.append(extra)
    return items


def _build(context):
    pkg_share = get_package_share_directory('eys3d_camera')
    dm_quality_cfg_dir = os.path.join(pkg_share, 'cfg', 'DM_Quality_Cfg')
    filter_profile_yaml = os.path.join(
        pkg_share, 'cfg', 'filter_profiles', f'{FILTER_PROFILE}.yaml')

    camera_params = {
        'model':              MODEL,
        'mode_id':            MODE_ID,
        'camera_name':        LaunchConfiguration('camera_name'),
        'dev_serial_number':  LaunchConfiguration('dev_serial_number'),
        'usb_port':           LaunchConfiguration('usb_port'),
        'dm_quality_cfg_dir': dm_quality_cfg_dir,
        # Declared bool on the node; state the type so the composable
        # dispatch does not hand the driver a string.
        'selfcal_enable':     ParameterValue(
            LaunchConfiguration('selfcal_enable'), value_type=bool),
    }

    ipc = LaunchConfiguration('use_intra_process_comms')

    descriptions = [
        ComposableNode(
            package='eys3d_camera',
            plugin='eys3d_camera::CameraNode',
            name='eys3d_camera',
            namespace=LaunchConfiguration('camera_name'),
            parameters=_camera_parameter_list(
                context, filter_profile_yaml, camera_params),
            extra_arguments=[{'use_intra_process_comms': ipc}],
        ),
        # Append downstream ComposableNode entries here. Set
        # extra_arguments=[{'use_intra_process_comms': ipc}] on each
        # one so it shares the same delivery path as CameraNode.
    ]

    container = ComposableNodeContainer(
        name=LaunchConfiguration('container_name'),
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=descriptions,
        output='screen',
    )

    # Camera model publisher, namespaced by camera_name so the topic never
    # clobbers another robot's /robot_description. robot_state_publisher is
    # not a component, so it runs beside the container rather than inside it.
    urdf_xacro = os.path.join(pkg_share, 'urdf', 'eys3d_camera.urdf.xacro')
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=LaunchConfiguration('camera_name'),
        output='log',
        condition=IfCondition(LaunchConfiguration('urdf')),
        parameters=[{
            'robot_description': ParameterValue(Command([
                'xacro ', urdf_xacro,
                ' model:=', MODEL,
                ' camera_name:=', LaunchConfiguration('camera_name'),
            ]), value_type=str),
        }],
    )
    return [container, rsp_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_name', default_value='G100P_1',
            description='ROS namespace and frame-id prefix. Matches the '
                        'default used by G100P.launch.py so RViz layouts '
                        'and recorded bags carry across launch styles.'),
        DeclareLaunchArgument(
            'container_name', default_value='eys3d_container',
            description='Name of the ComposableNodeContainer that hosts '
                        'CameraNode and any downstream components.'),
        DeclareLaunchArgument(
            'usb_port', default_value='',
            description='Bind to a specific camera by sysfs USB '
                        'interface path (e.g. "2-1:1.0").'),
        DeclareLaunchArgument(
            'dev_serial_number', default_value='',
            description='Bind to a specific camera by serial-number '
                        'substring.'),
        DeclareLaunchArgument(
            'use_intra_process_comms', default_value='false',
            description='Enable rclcpp intra-process delivery on every '
                        'driver topic except /tf_static (which is held '
                        'on DDS for portability). Subscribers in the '
                        'same container receive a unique_ptr instead of '
                        'a deserialised copy.'),
        DeclareLaunchArgument(
            'selfcal_enable', default_value='false',
            description='Enable in-stream self-calibration, exposing the '
                        '<camera_name>/selfcal/run action and '
                        '<camera_name>/selfcal/commit service. Only one '
                        'session may run per process, so cameras sharing '
                        'this container calibrate one at a time.'),
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='Optional YAML file appended to the CameraNode '
                        'parameters list. Applied after the filter '
                        'profile and the explicit launch arguments; '
                        'later entries override earlier ones.'),
        DeclareLaunchArgument(
            'urdf', default_value='true',
            description='Publish the camera model to '
                        '<camera_name>/robot_description via '
                        'robot_state_publisher. Set false for the driver '
                        'and container alone.'),
        OpaqueFunction(function=_build),
    ])
